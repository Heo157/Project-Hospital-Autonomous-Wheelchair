#!/usr/bin/env python3
"""
============================================================================
 파일명: tcp_bridge.py (Final Fixed Version)
 설명:   ROS 2(Nav2) <-> TCP(C Server) 간의 통신 중계 및 로봇 FSM 제어기
 수정사항:
   1. 실행 인자(sys.argv)로 로봇 이름 수신 (서버와 이름 불일치 해결)
   2. Float32 메시지 타입 Import 추가 (NameError 해결)
   3. 상세 로그 출력 적용
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

# -------------------------------------------------------------------------
# [ROS 2 메시지 타입 임포트]
# -------------------------------------------------------------------------
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
# [수정] Float32 추가 (필수)
from std_msgs.msg import Int32, Bool, String, Float32

# =========================================================================
# 1. 통신 프로토콜 및 상수 정의
# =========================================================================
MAGIC_NUMBER = 0xAB
DEVICE_ROBOT_ROS = 0x02

MSG_LOGIN_REQ   = 0x01
MSG_ROBOT_STATE = 0x20
MSG_ASSIGN_GOAL = 0x30

STATE_WAITING  = 0
STATE_HEADING  = 1
STATE_BOARDING = 2
STATE_RUNNING  = 3
STATE_STOP     = 4
STATE_ARRIVED  = 5
STATE_EXITING  = 6
STATE_CHARGING = 7
STATE_ERROR    = 99

HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)
STATE_FMT = "<ifffBiB" 
STATE_SIZE = struct.calcsize(STATE_FMT)
GOAL_FMT = "<iffff64s" 
GOAL_SIZE = struct.calcsize(GOAL_FMT)

BTN_BOARDING_COMPLETE = 2
BTN_RESUME = 3
BTN_EMERGENCY = 4
BTN_EXIT_COMPLETE = 5

# =========================================================================
# 2. 길찾기 전담 클래스 (A* 알고리즘)
# =========================================================================
class SimplePathFinder:
    def __init__(self, json_path):
        self.nodes = {}
        self.edges = {}
        self.load_map(json_path)

    def load_map(self, json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 노드 정보 로드
            self.nodes = {int(k): tuple(v) for k, v in data.get('nodes', {}).items()}
            
            # 엣지 정보 로드
            self.edges = {}
            edge_count = 0
            # JSON 포맷에 따라 'edges' 키 처리 (리스트 [[u,v,w], ...] 형태 가정)
            raw_edges = data.get('edges', [])
            for item in raw_edges:
                if len(item) >= 3:
                    u, v, w = item[0], item[1], item[2]
                    self.edges.setdefault(u, []).append((v, w))
                    self.edges.setdefault(v, []).append((u, w))
                    edge_count += 1
            
            print(f"🗺️  Map Loaded: {len(self.nodes)} nodes, {edge_count} edges from '{json_path}'")
            
        except Exception as e:
            print(f"⚠️  Map load warning: {e}")
            self.nodes = {}

    def find_nearest_node(self, target_x, target_y):
        if not self.nodes: return None
        return min(self.nodes.keys(), key=lambda k: math.dist((target_x, target_y), self.nodes[k]))

    def get_path(self, start_x, start_y, goal_x, goal_y):
        if not self.nodes: return [(goal_x, goal_y)]

        start_node = self.find_nearest_node(start_x, start_y)
        end_node = self.find_nearest_node(goal_x, goal_y)

        if start_node is None or end_node is None:
            return [(goal_x, goal_y)]

        queue = [(0, start_node, [])]
        visited = set()

        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            
            if curr in visited: continue
            visited.add(curr)
            
            new_path = path + [curr]
            
            if curr == end_node:
                # 노드 좌표들 변환 + 최종 좌표
                return [self.nodes[n] for n in new_path] + [(goal_x, goal_y)]
            
            for neighbor, weight in self.edges.get(curr, []):
                if neighbor not in visited:
                    # Heuristic: 직선 거리
                    h = math.dist(self.nodes[neighbor], self.nodes[end_node])
                    heapq.heappush(queue, (cost + weight + h, neighbor, new_path))
        
        return [(goal_x, goal_y)]

# =========================================================================
# 3. 메인 ROS 노드 클래스
# =========================================================================
class TcpBridge(Node):
    # [수정] __init__에서 robot_name과 map_file을 인자로 받음
    def __init__(self, robot_name_arg, map_file_arg):
        super().__init__("tcp_bridge")

        # 1. 파라미터 설정
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = 8080
        
        # [중요] 인자로 받은 이름 사용
        self.robot_name = robot_name_arg
        self.map_file = map_file_arg

        # 2. 내부 변수 초기화
        self.current_state = STATE_WAITING
        self.prev_state = STATE_WAITING
        self.mission_mode = "NONE"

        self.battery_percent = 100
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.ultra_distance = 0
        self.seat_detected = False
        self.current_caller = ""

        # 네비게이션 관련
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0
        self.waypoint_queue = []

        # TCP 관련
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        self.backoff = 1.0
        self.next_connect_time = 0.0

        # 길찾기 객체
        self.pathfinder = SimplePathFinder(self.map_file)

        # 3. ROS 통신 설정 (Namespace 적용)
        prefix = f"/{self.robot_name}"
        
        # Subscriber
        self.create_subscription(Odometry, f"{prefix}/odom", self.odom_pose_cb, 10)
        self.create_subscription(BatteryState, f"{prefix}/battery_state", self.batt_cb, 10)
        # [수정] Float32 타입 사용
        self.create_subscription(Float32, f"{prefix}/ultra_distance_cm", self.ultra_cb, 10)
        self.create_subscription(Bool, f"{prefix}/seat_detected", self.seat_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/button", self.button_cb, 10)

        # Publisher
        self.ui_pub = self.create_publisher(String, f"{prefix}/ui/info", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10) # Nav2 Goal은 전역 토픽 사용
        self.caller_pub = self.create_publisher(String, f"{prefix}/caller_name", 10)

        # Timer & Thread
        self.create_timer(0.5, self.tx_timer_cb) # 2Hz 전송
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"========================================")
        self.get_logger().info(f"🚀 TCP Bridge Started")
        self.get_logger().info(f"🤖 Robot Name: {self.robot_name}")
        self.get_logger().info(f"🗺️  Map File: {self.map_file}")
        self.get_logger().info(f"📡 Server: {self.server_ip}:{self.server_port}")
        self.get_logger().info(f"========================================")

    # --- Callbacks ---
    def odom_pose_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Quaternion to Yaw (Simple)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def batt_cb(self, msg): 
        self.battery_percent = int(msg.percentage) if msg.percentage > 1.0 else int(msg.percentage * 100)

    def ultra_cb(self, msg): 
        self.ultra_distance = int(msg.data) # Float32 -> Int 변환

    def seat_cb(self, msg): 
        self.seat_detected = msg.data

    def button_cb(self, msg):
        btn = msg.data
        self.get_logger().info(f"🔘 Button Clicked: {btn}")
        self.handle_button_logic(btn)

    # --- Logic (FSM) ---
    def change_state(self, new_state):
        if self.current_state != new_state:
            self.get_logger().info(f"🔄 State Change: {self.current_state} -> {new_state}")
            self.current_state = new_state
        self.publish_ui_info()

    def handle_button_logic(self, btn):
        if self.current_state == STATE_BOARDING:
            if btn == BTN_BOARDING_COMPLETE:
                if self.seat_detected:
                    self.get_logger().info("✅ 탑승 완료 확인. 목적지로 이동합니다.")
                    self.change_state(STATE_RUNNING)
                    self.mission_mode = "DELIVER"
                    # 저장해둔 최종 목적지로 이동
                    self.start_path_navigation(self.final_goal_x, self.final_goal_y)
                else:
                    self.get_logger().warn("⚠️ 탑승 버튼 눌림, 그러나 환자 미감지 (FSR Fail)")

        elif self.current_state == STATE_RUNNING:
            if btn == BTN_EMERGENCY:
                self.get_logger().warn("🚨 비상 정지 버튼 눌림!")
                self.prev_state = self.current_state
                self.change_state(STATE_STOP)
                self.stop_nav2()

        elif self.current_state == STATE_STOP:
            if btn == BTN_RESUME:
                self.get_logger().info("▶️ 동작 재개 버튼 눌림.")
                self.change_state(self.prev_state)
                # 멈췄던 곳(혹은 현재 목표)으로 다시 이동 명령
                self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

        elif self.current_state == STATE_ARRIVED:
            if btn == BTN_EXIT_COMPLETE:
                if not self.seat_detected:
                    self.get_logger().info("✅ 하차 완료 확인. 대기 모드로 전환.")
                    self.change_state(STATE_WAITING)
                    self.mission_mode = "NONE"
                    self.current_caller = ""
                    self.publish_ui_info()
                else:
                    self.get_logger().warn("⚠️ 하차 버튼 눌림, 그러나 환자 감지됨 (FSR Check)")

    # --- Nav2 Helper ---
    def publish_nav2_goal(self, x, y):
        if self.current_state == STATE_STOP: return
        self.current_goal_x = float(x)
        self.current_goal_y = float(y)
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0 # No rotation preference
        self.goal_pub.publish(goal)

    def stop_nav2(self):
        self.waypoint_queue = []
        self.publish_nav2_goal(self.x, self.y) # 현재 위치로 목표 설정 = 정지

    def start_path_navigation(self, tx, ty):
        path = self.pathfinder.get_path(self.x, self.y, tx, ty)
        self.get_logger().info(f"🚗 Path Planned: {len(path)} waypoints to ({tx:.2f}, {ty:.2f})")
        self.waypoint_queue = path
        self.pop_and_drive()

    def pop_and_drive(self):
        if self.waypoint_queue:
            wp = self.waypoint_queue.pop(0)
            self.publish_nav2_goal(wp[0], wp[1])
        else:
            self.get_logger().info("🏁 경로 끝 도달 (Target Reached)")

    def publish_ui_info(self):
        # Format: "State@Caller@StartLoc@DestLoc"
        s_mode = str(self.current_state)
        s_caller = self.current_caller if self.current_caller else "Waiting"
        # 장소 이름은 맵 매니저가 있으면 좋지만, 여기선 일단 비움
        s_start = "-" 
        s_dest = "-"
        msg = f"{s_mode}@{s_caller}@{s_start}@{s_dest}"
        self.ui_pub.publish(String(data=msg))

    # --- Network (TCP) ---
    def handle_server_message(self, msg_type, payload):
        if msg_type == MSG_ASSIGN_GOAL:
            if len(payload) != GOAL_SIZE: return
            order, sx, sy, gx, gy, raw_name = struct.unpack(GOAL_FMT, payload)
            
            try: caller = raw_name.split(b'\x00')[0].decode('utf-8')
            except: caller = "Unknown"
            
            # 99: 자폭 명령
            if order == 99: 
                self.get_logger().warn("💀 Received KILL command from Server.")
                self.destroy_node()
                sys.exit(0)

            self.get_logger().info(f"📩 Order Received: {order} from '{caller}'")
            self.current_caller = caller
            self.caller_pub.publish(String(data=caller))
            
            # 최종 목적지 저장
            self.final_goal_x = gx
            self.final_goal_y = gy

            if order == 6: # 배차 (환자에게 이동)
                self.change_state(STATE_HEADING)
                self.mission_mode = "PICKUP"
                self.start_path_navigation(sx, sy)
            elif order in [1, 4, 5]: # 단순 이동
                self.change_state(STATE_RUNNING)
                self.mission_mode = "MOVE"
                self.start_path_navigation(gx, gy)

    def tx_timer_cb(self):
        if not self.connect(): return
        self.send_login_once()
        self.publish_ui_info()

        # 도착 판정 (단순 거리 계산)
        dist = math.dist((self.x, self.y), (self.current_goal_x, self.current_goal_y))
        
        # 이동 중이고 목표 근처(0.3m)에 왔다면
        if self.current_state in [STATE_HEADING, STATE_RUNNING] and dist < 0.3:
            if self.waypoint_queue:
                self.pop_and_drive() # 다음 경유지로
            else:
                # 큐가 비었으면 진짜 도착
                if self.current_state == STATE_HEADING:
                    self.get_logger().info("🏁 출발지(환자 위치) 도착")
                    self.change_state(STATE_BOARDING)
                elif self.current_state == STATE_RUNNING:
                    self.get_logger().info("🏁 목적지 도착")
                    self.change_state(STATE_ARRIVED)

        # 상태 보고 패킷 전송
        try:
            payload = struct.pack(STATE_FMT,
                int(self.battery_percent), self.x, self.y, self.theta,
                int(self.current_state),
                int(self.ultra_distance), int(1 if self.seat_detected else 0)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)
        except Exception as e:
            self.close_socket(f"TX Error: {e}")

    # --- Socket Utils ---
    def connect(self):
        now = time.time()
        if now < self.next_connect_time: return False
        with self.lock:
            if self.sock: return True
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None)
                self.sock = s; self.logged_in = False; self.backoff = 1.0; self.next_connect_time = 0.0
                self.get_logger().info(f"✅ Connected to Server")
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
        self.get_logger().warn(f"🔌 Socket Closed: {reason}")

    def send_packet(self, msg_type, payload):
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        with self.lock:
            if not self.sock: return
            try: self.sock.sendall(header + payload)
            except Exception as e: self.close_socket(f"Send Err: {e}")

    def send_login_once(self):
        if self.logged_in: return
        # [중요] self.robot_name을 사용하여 로그인
        name_bytes = self.robot_name.encode("utf-8")[:64]
        self.send_packet(MSG_LOGIN_REQ, name_bytes)
        self.logged_in = True
        self.get_logger().info(f"🔑 Login Request sent as '{self.robot_name}'")

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

# =========================================================================
# 4. 메인 실행부 (여기가 핵심 수정됨)
# =========================================================================
def main():
    rclpy.init()

    # 기본값 설정
    robot_name = "wc1"
    map_file = "map_graph.json"

    # [수정] sys.argv로 인자 받기 (Server가 보내주는 인자 처리)
    # 예: python3 tcp_bridge.py turtlebot3 map_graph.json
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    if len(sys.argv) > 2:
        map_file = sys.argv[2]

    # 노드 생성 시 인자 전달
    node = TcpBridge(robot_name, map_file)

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