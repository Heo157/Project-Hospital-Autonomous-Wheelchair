"""
============================================================================
 파일명: tcp_bridge.py
 설명:   ROS 2(Nav2) <-> TCP(C Server) 간의 통신 중계 및 로봇 FSM 제어기
 수정일: 2025-12-29
 작성자: Team Hospital / AI Assistant

 [시스템 개요]
 이 노드는 로봇(TurtleBot) 내부에서 실행되며, 외부의 C언어 관제 서버와 
 TCP/IP로 통신하여 로봇을 제어하고 상태를 보고하는 역할을 합니다.

 [주요 기능]
 1. TCP 클라이언트: C 서버(Port 8080)에 접속하고 끊기면 재접속합니다.
 2. 상태 보고: 로봇의 위치, 배터리, 센서(초음파, 착석) 값을 C 서버로 보냅니다.
 3. 명령 수행: C 서버에서 온 이동 명령(Order)을 받아 Nav2에게 전달합니다.
 4. 호출자 표시: 배차 명령 시 누가 불렀는지(호출자 이름)를 받아 화면에 띄웁니다.
 5. 자동 관리: DB에서 로봇이 삭제되면 서버 명령(99)을 받아 스스로 종료합니다.
============================================================================
"""

import sys
import socket
import struct
import threading
import time
import math
import json
import heapq

# -------------------------------------------------------------------------
# [ROS 2 관련 라이브러리 임포트]
# -------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient

# -------------------------------------------------------------------------
# [ROS 2 메시지 타입 임포트]
# -------------------------------------------------------------------------
# PoseWithCovarianceStamped: AMCL 위치 정보 (공분산 포함)
# PoseStamped: Nav2 목표 지점 명령
# Odometry: 휠 인코더 기반 위치 정보
# BatteryState: 배터리 잔량
# Int32, Bool, String: 초음파 거리, 착석 여부, 호출자 이름 등 단순 데이터
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool, String
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

# =========================================================================
# 1. 통신 프로토콜 및 상수 정의
# (주의: 이 값들은 C 서버의 common_defs.h와 반드시 일치해야 합니다)
# =========================================================================

# 패킷 유효성 검사 키 (Header Start)
MAGIC_NUMBER = 0xAB

# 장치 식별 ID (0x02: ROS Robot)
DEVICE_ROBOT_ROS = 0x02

# 메시지 타입 (명령 종류)
MSG_LOGIN_REQ   = 0x01  # "저 접속합니다" (로그인 요청)
MSG_ROBOT_STATE = 0x20  # "제 상태는 이렇습니다" (상태 보고)
MSG_ASSIGN_GOAL = 0x30  # "어디로 가라" (목표 명령)

# 로봇 FSM(Finite State Machine) 상태 상수
STATE_WAITING  = 0   # 대기 중 (명령 없음)
STATE_HEADING  = 1   # 환자에게 가는 중 (Start 지점으로 이동)
STATE_BOARDING = 2   # 환자 탑승 대기 중 (도착 후 잠시 멈춤)
STATE_RUNNING  = 3   # 목적지로 이동 중 (Goal 지점으로 이동)
STATE_STOP     = 4   # 비상 정지 상태
STATE_ARRIVED  = 5   # 최종 목적지 도착 완료
STATE_EXITING  = 6   # 환자 하차 대기 중
STATE_CHARGING = 7   # 충전 중
STATE_ERROR    = 99  # 에러 상태

# -------------------------------------------------------------------------
# [패킷 구조체 포맷 정의 (struct 모듈 사용)]
# <: 리틀 엔디안 (Intel/ARM 표준)
# B: unsigned char (1 byte), i: int (4 bytes), f: float (4 bytes), s: char[]
# -------------------------------------------------------------------------

# 1. 헤더 포맷 (4바이트)
# [Magic(1)][Device(1)][Type(1)][PayloadLen(1)]
HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)

# 2. 상태(State) 패킷 포맷 (22바이트) -> 로봇이 서버로 보냄
# [Battery(4)][X(4)][Y(4)][Theta(4)][State(1)][Ultra(4)][Seat(1)]
STATE_FMT = "<ifffBiB" 
STATE_SIZE = struct.calcsize(STATE_FMT)

# 3. 목표(Goal) 패킷 포맷 (84바이트) -> 서버가 로봇에게 보냄
# [Order(4)][StartX(4)][StartY(4)][GoalX(4)][GoalY(4)][CallerName(64)]
# 64s: 64바이트 고정 길이 문자열
GOAL_FMT = "<iffff64s" 
GOAL_SIZE = struct.calcsize(GOAL_FMT)

# =========================================================================
# 1. 길찾기 전담 클래스 (A* 알고리즘)
# =========================================================================
class SimplePathFinder:
    def __init__(self, json_path):
        self.nodes = {}
        self.edges = {}
        self.load_map(json_path)

    def load_map(self, json_path):
        """ JSON 파일에서 노드와 엣지(거리 포함) 정보를 로드 """
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            # 1. 노드 정보: "1": [x, y] -> {1: (x, y)} 변환
            self.nodes = {int(k): tuple(v) for k, v in data['nodes'].items()}
            
            # 2. 엣지 정보: [u, v, dist] (C 서버가 계산한 거리 사용)
            self.edges = {}
            for u, v, w in data['edges']:
                self.edges.setdefault(u, []).append((v, w))
                self.edges.setdefault(v, []).append((u, w)) # 양방향 그래프
            
            print(f"🗺️  Map Loaded: {len(self.nodes)} nodes, {len(data['edges'])} edges")
            
        except FileNotFoundError:
            print(f"⚠️  Map file not found: {json_path}")
        except Exception as e:
            print(f"❌ Map Load Error: {e}")

    def find_nearest_node(self, target_x, target_y):
        """ 주어진 좌표에서 가장 가까운 노드 ID 찾기 """
        if not self.nodes: return None
        return min(self.nodes.keys(), key=lambda k: math.dist((target_x, target_y), self.nodes[k]))

    def get_path(self, start_x, start_y, goal_x, goal_y):
        """ 
        (Start_x, Start_y) -> (Goal_x, Goal_y) 로 가는 웨이포인트 경로 계산
        반환값: [(wp1_x, wp1_y), (wp2_x, wp2_y), ..., (goal_x, goal_y)]
        """
        # 1. 맵 데이터가 없으면 그냥 직선 경로 반환
        if not self.nodes:
            return [(goal_x, goal_y)]

        # 2. 시작점/끝점과 가장 가까운 노드 매칭
        start_node = self.find_nearest_node(start_x, start_y)
        end_node = self.find_nearest_node(goal_x, goal_y)

        if start_node is None or end_node is None:
            return [(goal_x, goal_y)]

        # 3. A* 알고리즘 수행
        queue = [(0, start_node, [])] # (비용, 현재노드, 경로리스트)
        visited = set()
        final_node_path = []

        found = False
        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            
            if curr in visited: continue
            visited.add(curr)
            
            # 경로 업데이트
            path = path + [curr]
            
            # 목적지 도착
            if curr == end_node:
                final_node_path = path
                found = True
                break
            
            # 이웃 탐색
            for neighbor, weight in self.edges.get(curr, []):
                if neighbor not in visited:
                    # 휴리스틱: 현재 비용 + 이웃까지 거리 + 이웃에서 목적지까지 직선거리
                    priority = cost + weight + math.dist(self.nodes[neighbor], self.nodes[end_node])
                    heapq.heappush(queue, (cost + weight, neighbor, path))
        
        # 4. 결과 좌표 리스트 생성
        result_path = []
        
        if found:
            # (옵션) 첫 번째 노드가 내 위치랑 너무 가까우면(0.5m 이내) 스킵 가능
            # 여기서는 안전하게 다 포함시킴
            for node_id in final_node_path:
                result_path.append(self.nodes[node_id])
        
        # 5. 마지막에 '진짜 목표 좌표' 추가 (노드 위치랑 미세하게 다를 수 있으므로)
        result_path.append((goal_x, goal_y))
        
        return result_path

# =========================================================================
# 2. 메인 ROS 노드 클래스
# =========================================================================
class TcpBridge(Node):
    def __init__(self, parameter_overrides=None, map_file="map_graph.json"):
        super().__init__("tcp_bridge", parameter_overrides=parameter_overrides)

        # ... (파라미터 초기화 기존과 동일) ...
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)
        self.robot_name = self.declare_parameter("robot_name", "wc1").value
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value)
        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        prefix = f"/{self.robot_name}"
        
        # 토픽 설정
        self.topic_amcl = self.declare_parameter("topic_amcl", f"{prefix}/amcl_pose").value
        self.topic_odom = self.declare_parameter("topic_odom", f"{prefix}/odom").value
        self.topic_battery = self.declare_parameter("topic_battery", f"{prefix}/battery_state").value
        self.topic_ultra = self.declare_parameter("topic_ultra", f"{prefix}/ultra_distance_cm").value
        self.topic_seat = self.declare_parameter("topic_seat", f"{prefix}/seat_detected").value
        self.topic_cmd_vel = self.declare_parameter("topic_cmd_vel", f"{prefix}/cmd_vel").value
        self.topic_goal = self.declare_parameter("topic_goal", "/goal_pose").value
        self.topic_caller = self.declare_parameter("topic_caller", f"{prefix}/caller_name").value


        self.cmd_vel_pub = self.create_publisher(Twist, self.topic_cmd_vel, 10)
        # 내부 변수
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.battery_percent = 90
        self.ultra_distance = 0; self.seat_detected = False
        self.current_caller = ""

        self.current_state = STATE_WAITING
        self.prev_state = STATE_WAITING
        self.mission_mode = "NONE" 
        
        # 웨이포인트 주행을 위한 변수들
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.waypoint_queue = [] # [(x1,y1), (x2,y2)...]
        self.last_goal = None     # (x, y) 마지막 이동 목표
        self.paused_goal = None   # (x, y) STOP 시 재개할 목표
        
        self.goal_active = False
        self.paused_queue = []
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0


        # TCP 소켓
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        self.backoff = 1.0
        self.next_connect_time = 0.0

        # [New] 길찾기 객체 생성
        self.pathfinder = SimplePathFinder(map_file)

        # ROS 통신 설정 (구독/발행)
        if self.use_amcl_pose:
            amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PoseWithCovarianceStamped, self.topic_amcl, self.pose_cb, amcl_qos)
        else:
            self.create_subscription(Odometry, self.topic_odom, self.odom_pose_cb, 10)

        self.create_subscription(BatteryState, self.topic_battery, self.batt_cb, 10)
        self.create_subscription(Int32, self.topic_ultra, self.ultra_cb, 10)
        self.create_subscription(Bool, self.topic_seat, self.seat_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.topic_goal, 10)
        self.caller_pub = self.create_publisher(String, self.topic_caller, 10)

        # 타이머 및 스레드
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)
        
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"🚀 Bridge Started for [{self.robot_name}]")
        self.get_logger().info(f"📂 Map File: {map_file}")

    # ... [섹션 A, B 콜백 함수들은 기존과 동일 (생략 없음)] ...
    
    def pose_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_pose_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def batt_cb(self, msg):
        if msg.percentage is not None and msg.percentage >= 0.0:
            p = int(msg.percentage * 100.0) if msg.percentage <= 1.0 else int(msg.percentage)
            self.battery_percent = max(0, min(100, p))

    def ultra_cb(self, msg):
        self.ultra_distance = msg.data

    def seat_cb(self, msg):
        self.seat_detected = msg.data

    def get_state_name(self, state_id):
        names = { STATE_WAITING: "WAITING", STATE_HEADING: "HEADING", STATE_BOARDING: "BOARDING",
            STATE_RUNNING: "RUNNING", STATE_STOP: "STOP", STATE_ARRIVED: "ARRIVED",
            STATE_EXITING: "EXITING", STATE_CHARGING: "CHARGING", STATE_ERROR: "ERROR" }
        return names.get(state_id, "UNKNOWN")

    def change_state(self, new_state):
        if self.current_state != new_state:
            old = self.get_state_name(self.current_state)
            new = self.get_state_name(new_state)
            self.get_logger().info(f"[State Change] {old} -> {new}")
            self.current_state = new_state

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.z = math.sin(yaw * 0.5)
        return q

    # =========================================================================
    # [섹션 C] Nav2 제어 (수정됨: 큐 처리 추가)
    # =========================================================================

    def publish_nav2_goal(self, x, y):
        """ Nav2에게 '현재' 목표 좌표 전송 """
        if self.current_state == STATE_STOP:
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation = self.yaw_to_quaternion(0.0)

        self.goal_pub.publish(goal)
        self.current_goal_x = float(x)
        self.current_goal_y = float(y)
        self.goal_active = True
        # 로그는 너무 자주 찍히면 정신없으니 주석 처리 혹은 필요시 해제
        # self.get_logger().info(f"Nav2 Goal -> ({x:.2f}, {y:.2f})")

    def _cancel_nav2_goals_best_effort(self):
        """Nav2 액션 goal 전체 취소(가능하면)."""
        try:
            if self.nav2_client is None:
                return
            if not self.nav2_client.server_is_ready():
                # 액션 서버가 준비 안 되었으면 취소 시도 자체가 의미 없음
                return

            cancel_future = self.nav2_client.cancel_all_goals_async()

            # rx_thread에서 돌기 때문에 오래 기다리면 위험 -> 짧게만 대기
            start = time.time()
            while not cancel_future.done() and (time.time() - start) < 0.5:
                time.sleep(0.01)
        except Exception as e:
            self.get_logger().warn(f"[STOP] cancel_all_goals best-effort failed: {e}")



    def stop_nav2(self):
        """로봇 긴급 정지"""
        self.get_logger().warn("[STOP] stop_nav2()")

        # 1) Nav2 goal 취소 (Action 기반일 때 의미)
        self._cancel_nav2_goals_best_effort()

        # 2) cmd_vel=0 반복 전송 (토픽이 맞아야 실제로 멈춤)
        tw = Twist()
        for _ in range(30):
            self.cmd_vel_pub.publish(tw)
            time.sleep(0.01)

        # 3) 현재 위치를 goal로 한번 더(관성/플래너 영향 완화 목적)
        if self.x is not None and self.y is not None:
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = "map"
            g.pose.position.x = float(self.x)
            g.pose.position.y = float(self.y)
            g.pose.orientation = self.yaw_to_quaternion(self.theta)

            for _ in range(3):
                self.goal_pub.publish(g)
                time.sleep(0.03)

        self.get_logger().warn("[STOP] done")


    def pop_and_drive(self):
        """ 큐에서 다음 웨이포인트를 꺼내서 이동 """
        if self.waypoint_queue:
            next_wp = self.waypoint_queue.pop(0)
            self.get_logger().info(f"🚗 다음 경유지로 이동: {next_wp}")
            self.publish_nav2_goal(next_wp[0], next_wp[1])
        else:
            self.get_logger().info("🏁 모든 경로 소진 (도착)")

    def step_boarding_complete(self):
        self.get_logger().info("✅ 탑승 완료. 목적지로 이동합니다.")
        self.change_state(STATE_RUNNING)
        self.mission_mode = "DELIVER"
        # 배달(목적지) 경로 재계산 (내 위치 -> 최종 목적지)
        # *최종 목적지 좌표는 self.final_goal_x, y에 저장되어 있음*
        self.start_path_navigation(self.final_goal_x, self.final_goal_y)

    def step_exiting_complete(self):
        self.get_logger().info("✅ 하차 완료. 임무 종료.")
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"
        self.current_caller = ""
        self.caller_pub.publish(String(data="Waiting..."))

    # [New] 경로 생성 및 주행 시작 헬퍼 함수
    def start_path_navigation(self, target_x, target_y):
        # 1. 경로 계산 (내 위치 -> 목표 위치)
        path = self.pathfinder.get_path(self.x, self.y, target_x, target_y)
        self.get_logger().info(f"Path Plan: {len(path)} waypoints")
        
        # 2. 큐에 등록
        self.waypoint_queue = path
        
        # 3. 첫 번째 지점으로 출발
        self.pop_and_drive()

    # =========================================================================
    # [섹션 D] 서버 메시지 처리 (수정됨)
    # =========================================================================

    def handle_server_message(self, msg_type, payload):
        if msg_type == MSG_ASSIGN_GOAL:
            if len(payload) != GOAL_SIZE: return
            order, sx, sy, gx, gy, raw_name = struct.unpack(GOAL_FMT, payload)
            
            try: caller_name_str = raw_name.split(b'\x00')[0].decode('utf-8')
            except: caller_name_str = "Unknown"
            
            if order == 99: # 자폭
                self.close_socket("Kill Cmd")
                self.destroy_node()
                sys.exit(0)

            self.get_logger().info(f"CMD 수신: Order={order}, Caller='{caller_name_str}'")
            if caller_name_str:
                self.current_caller = caller_name_str
                self.caller_pub.publish(String(data=self.current_caller))

            # ---------------------------------------------------------
            # [수정] 이동 로직: start_path_navigation 호출로 변경
            # ---------------------------------------------------------
            if order == 6: # [배차] Start 지점으로 이동
                # 최종 목적지 기억해둠 (탑승 후 사용)
                self.final_goal_x = gx
                self.final_goal_y = gy
                
                self.change_state(STATE_HEADING)
                self.mission_mode = "PICKUP"
                
                # Start 지점(sx, sy)까지 경로 주행 시작
                self.start_path_navigation(sx, sy)

            elif order in [1, 4, 5]: # [단순 이동 / 충전]
                self.change_state(STATE_RUNNING)
                self.mission_mode = "CHARGE" if order == 5 else "NONE"
                self.start_path_navigation(gx, gy)

            elif order == 2: # [비상 정지]
                self.get_logger().warn("🛑 Order 2 Received: EMERGENCY STOP")
                
                # 재개용 데이터 저장
                if self.current_state != STATE_STOP:
                    self.prev_state = self.current_state
                    
                    # 현재 목표 저장
                    # 현재 목표 저장 (좌표가 0일 수도 있으니 truthy 체크 금지)
                    if self.goal_active:
                        self.paused_goal = (float(self.current_goal_x), float(self.current_goal_y))
                    else:
                        self.paused_goal = None

                    
                    # 남은 웨이포인트 저장
                    self.paused_queue = self.waypoint_queue.copy()
                
                # 즉시 정지 실행
                self.stop_nav2()
                self.waypoint_queue.clear()
                self.change_state(STATE_STOP)

            elif order == 3: # [동작 재개]
                self.get_logger().info("▶️ Order 3 Received: RESUME")
                
                if self.paused_goal:
                    # 이전 상태 복원
                    prev = self.prev_state if self.prev_state != STATE_STOP else STATE_RUNNING
                    self.change_state(prev)
                    
                    # 경로 복원
                    if hasattr(self, 'paused_queue') and self.paused_queue:
                        self.get_logger().info(f"📍 경로 복원: {len(self.paused_queue)}개 웨이포인트")
                        self.waypoint_queue = self.paused_queue
                        self.pop_and_drive()
                    else:
                        # 큐 없으면 마지막 목표로
                        gx, gy = self.paused_goal
                        self.publish_nav2_goal(gx, gy)
                    
                    # 초기화
                    self.paused_goal = None
                    self.paused_queue = []
                else:
                    self.get_logger().warn("⚠️ Order 3 무시: 저장된 목표 없음")

    # =========================================================================
    # [섹션 E] 데이터 전송 및 도착 판정 (수정됨: 웨이포인트 로직)
    # =========================================================================

    def tx_timer_cb(self):
        if not self.connect(): return
        self.send_login_once()

        try:
            # 도착 판정 거리 (0.3m)
            dist = math.sqrt((self.x - self.current_goal_x)**2 + (self.y - self.current_goal_y)**2)
            
            # 웨이포인트 통과 로직
            # 이동 중(HEADING/RUNNING)이고 목표에 가까워졌다면?
            if self.current_state in [STATE_HEADING, STATE_RUNNING] and dist < 0.5:
                
                if self.waypoint_queue:
                    # 1. 아직 갈 길이 남음 -> 다음 웨이포인트 꺼냄
                    self.get_logger().info("🚩 경유지 통과.")
                    self.pop_and_drive()
                else:
                    # 2. 큐가 비었음 -> "진짜 도착"
                    if self.current_state == STATE_HEADING:
                        self.get_logger().info("🏁 출발지 도착 (탑승 대기)")
                        self.change_state(STATE_BOARDING)
                        threading.Timer(5.0, self.step_boarding_complete).start()

                    elif self.current_state == STATE_RUNNING:
                        self.get_logger().info("🏁 최종 목적지 도착")
                        self.change_state(STATE_ARRIVED)
                        
                        if self.mission_mode == "DELIVER":
                            self.change_state(STATE_EXITING)
                            threading.Timer(5.0, self.step_exiting_complete).start()
                            self.mission_mode = "DONE_WAIT"
                        elif self.mission_mode == "CHARGE":
                            self.change_state(STATE_CHARGING)
                            self.mission_mode = "DONE_CHARGE"
                        else:
                            self.change_state(STATE_WAITING)

            # 서버 리포트 전송 (기존 동일)
            if self.current_caller:
                self.caller_pub.publish(String(data=self.current_caller))

            payload = struct.pack(STATE_FMT,
                int(self.battery_percent),
                float(self.x), float(self.y), float(self.theta),
                int(self.current_state),
                int(self.ultra_distance),
                int(1 if self.seat_detected else 0)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            self.get_logger().error(f"TX Error: {e}")
            self.close_socket("TX Error")

    # ... [소켓 유틸리티 함수들은 기존과 동일 (생략)] ...
    def _set_keepalive(self, s):
        try: s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except: pass
    
    def connect(self):
        now = time.time()
        if now < self.next_connect_time: return False
        with self.lock:
            if self.sock: return True
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0)
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None)
                self.sock = s; self.logged_in = False; self.backoff = 1.0; self.next_connect_time = 0.0
                return True
            except:
                if self.sock: s.close()
                self.sock = None; self.logged_in = False
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, 60.0)
                return False

    def close_socket(self, reason):
        with self.lock:
            if self.sock: 
                try: self.sock.close()
                except: pass
            self.sock = None; self.logged_in = False
        self.get_logger().warn(f"Closed: {reason}")

    def send_packet(self, msg_type, payload):
        if len(payload) > 255: return
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        with self.lock:
            if not self.sock: return
            try: self.sock.sendall(header + payload)
            except Exception as e: self.close_socket(f"Send Err: {e}")

    def send_login_once(self):
        if self.logged_in: return
        self.send_packet(MSG_LOGIN_REQ, self.robot_name.encode("utf-8")[:64])
        self.logged_in = True
        self.get_logger().info(f"Login: {self.robot_name}")

    def recvall(self, sock, n):
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk: return b""
                data += chunk
            except: return b""
        return data

    def rx_loop(self):
        while self.running and rclpy.ok():
            with self.lock: sock = self.sock
            if sock is None:
                time.sleep(1.0); continue
            try:
                hdr = self.recvall(sock, HDR_SIZE)
                if len(hdr) != HDR_SIZE:
                    self.close_socket("Header Err"); continue
                magic, dev, msg_type, length = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue
                
                payload = b""
                if length > 0:
                    payload = self.recvall(sock, length)
                    if len(payload) != length:
                        self.close_socket("Payload Err"); continue
                
                self.handle_server_message(msg_type, payload)
            except Exception as e:
                self.close_socket(f"RX Err: {e}"); time.sleep(1.0)


def main():
    rclpy.init()
    
    robot_name = "wc1"
    map_file = "map_graph.json" # [New] 맵 파일 경로 기본값

    # argv 처리: 1번은 로봇이름, 2번은 맵파일경로
    if len(sys.argv) > 1: robot_name = sys.argv[1]
    if len(sys.argv) > 2: map_file = sys.argv[2]

    node = TcpBridge(
        parameter_overrides=[Parameter("robot_name", Parameter.Type.STRING, robot_name)],
        map_file=map_file
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.close_socket("Shutdown")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()