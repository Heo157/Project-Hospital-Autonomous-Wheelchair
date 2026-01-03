#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================================
 파일명: tcp_bridge.py (Final Debug Version)
 설명:   ROS 2(Nav2) <-> TCP(C Server) 통신 및 로봇 FSM 제어기
 
 [수정 사항]
   1. 모든 주석 한글화 및 상세 설명 추가
   2. 주요 동작 단계마다 print() 문으로 실행 흐름 표시
   3. 버튼 입력 노이즈(0값) 출력 제거, 유효 입력 시에만 반응
   4. 웨이포인트 주행 시 멈춤 현상 해결 (Look-ahead 거리 적용)
   5. Admin 호출 및 정지/재개 로직 강화
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

# ROS 2 라이브러리
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# ROS 2 메시지 타입
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool, String, Float32

# =========================================================================
# 1. 통신 프로토콜 및 상수 정의
# =========================================================================
# 프로토콜 매직 넘버 (패킷 유효성 검사)
MAGIC_NUMBER = 0xAB
DEVICE_ROBOT_ROS = 0x02

# 메시지 타입 (서버와 약속된 번호)
MSG_LOGIN_REQ   = 0x01  # 로그인 요청
MSG_ROBOT_STATE = 0x20  # 로봇 상태 보고 (주기적)
MSG_ASSIGN_GOAL = 0x30  # 목적지 할당 명령 (서버 -> 로봇)

# 로봇 상태 (FSM)
STATE_WAITING  = 0  # 대기 중
STATE_HEADING  = 1  # 환자에게 가는 중 (픽업)
STATE_BOARDING = 2  # 환자 탑승 대기 (도착 후)
STATE_RUNNING  = 3  # 목적지로 이동 중 (이송/복귀)
STATE_STOP     = 4  # 비상 정지
STATE_ARRIVED  = 5  # 목적지 도착 (하차 대기)
STATE_EXITING  = 6  # 하차 완료 인사
STATE_CHARGING = 7  # 충전 중
STATE_ERROR    = 99 # 에러

# 패킷 포맷 (struct 라이브러리 포맷 스트링)
# 헤더: 매직넘버(1) + 디바이스ID(1) + 메시지타입(1) + 데이터길이(1)
HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)

# 상태 보고 패킷: 배터리(4), x(4), y(4), theta(4), 상태(1), 초음파(4), 착석(1)
STATE_FMT = "<ifffBiB" 
# 목적지 명령 패킷: 명령코드(4), 시작x(4), 시작y(4), 목표x(4), 목표y(4), 이름(64)
GOAL_FMT = "<iffff64s" 
GOAL_SIZE = struct.calcsize(GOAL_FMT)

# 버튼 값 매핑 (STM32에서 보내주는 값)
BTN_BOARDING_COMPLETE = 1  # 탑승 완료 (출발)
BTN_RESUME            = 3  # 주행 재개
BTN_EMERGENCY         = 4  # 비상 정지
BTN_EXIT_COMPLETE     = 5  # 하차 완료 (복귀)

# 주행 설정 (멈춤 현상 방지)
DIST_TOLERANCE_FINAL    = 0.2  # 최종 목적지는 0.2m 이내면 도착으로 인정
DIST_TOLERANCE_WAYPOINT = 0.7  # 중간 경유지는 1.2m 근처만 가도 통과 (부드러운 주행)

# =========================================================================
# 2. 길찾기 알고리즘 (A* PathFinder)
# =========================================================================
class SimplePathFinder:
    def __init__(self, json_path):
        self.nodes = {}
        self.edges = {}
        self.load_map(json_path)

    def load_map(self, json_path):
        """ JSON 맵 파일을 읽어서 노드와 간선 정보를 메모리에 저장합니다. """
        print(f"[Map] 맵 파일 로딩 시작: {json_path}")
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 노드 정보 파싱 (ID: (x, y))
            self.nodes = {int(k): tuple(v) for k, v in data.get('nodes', {}).items()}
            
            # 간선 정보 파싱
            self.edges = {}
            raw_edges = data.get('edges', [])
            for item in raw_edges:
                if len(item) >= 3:
                    u, v, w = item[0], item[1], item[2]
                    # 양방향 그래프로 저장
                    self.edges.setdefault(u, []).append((v, w))
                    self.edges.setdefault(v, []).append((u, w))
            
            print(f"[Map] 로딩 완료! 노드 {len(self.nodes)}개, 간선 정보 로드됨.")
        except Exception as e:
            print(f"[Map] ⚠️ 맵 로딩 실패: {e}")
            self.nodes = {}

    def find_nearest_node(self, tx, ty):
        """ 현재 좌표(tx, ty)에서 가장 가까운 맵 상의 노드를 찾습니다. """
        if not self.nodes: return None
        # 유클리드 거리 기준 최소값 탐색
        nearest = min(self.nodes.keys(), key=lambda k: math.dist((tx, ty), self.nodes[k]))
        return nearest

    def get_path(self, sx, sy, gx, gy):
        """ A* 알고리즘을 사용하여 최단 경로 좌표 리스트를 반환합니다. """
        print(f"[Path] 경로 탐색 요청: ({sx:.1f}, {sy:.1f}) -> ({gx:.1f}, {gy:.1f})")
        
        if not self.nodes: 
            print("[Path] 맵 노드가 없습니다. 직선 경로 반환.")
            return [(gx, gy)]

        # 시작점과 도착점에서 가장 가까운 노드 찾기
        start_node = self.find_nearest_node(sx, sy)
        end_node = self.find_nearest_node(gx, gy)

        if start_node is None or end_node is None:
            return [(gx, gy)]

        # A* 알고리즘 시작
        queue = [(0, start_node, [])] # (비용, 현재노드, 경로리스트)
        visited = set()
        
        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            
            if curr in visited: continue
            visited.add(curr)
            
            new_path = path + [curr]
            
            # 도착 노드 발견!
            if curr == end_node:
                print(f"[Path] 경로 탐색 성공! 총 {len(new_path)}개 경유지 생성.")
                # 노드 ID 리스트를 실제 좌표(x, y) 리스트로 변환하여 반환
                # 마지막에 최종 목적지 좌표 추가
                return [self.nodes[n] for n in new_path] + [(gx, gy)]
            
            # 인접 노드 탐색
            for neighbor, weight in self.edges.get(curr, []):
                if neighbor not in visited:
                    # 휴리스틱: 현재~목표까지의 직선 거리
                    h = math.dist(self.nodes[neighbor], self.nodes[end_node])
                    heapq.heappush(queue, (cost + weight + h, neighbor, new_path))
        
        print("[Path] 경로를 찾을 수 없음. 목적지 직행.")
        return [(gx, gy)]

# =========================================================================
# 3. 메인 ROS 노드 클래스
# =========================================================================
class TcpBridge(Node):
    def __init__(self, robot_name_arg, map_file_arg):
        super().__init__("tcp_bridge")
        print("\n" + "="*50)
        print(f"🚀 [Start] TCP Bridge 시작 (로봇명: {robot_name_arg})")
        print("="*50 + "\n")

        # --- 1. 설정 변수 초기화 ---
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = 8080
        self.robot_name = robot_name_arg
        self.map_file = map_file_arg

        # 로봇 상태 변수
        self.current_state = STATE_WAITING
        self.prev_state = STATE_WAITING   # 비상정지 전 상태 복구용
        self.mission_mode = "NONE"        # 현재 수행 중인 임무 (PICKUP, DELIVER, MOVE)

        # 센서 데이터 변수
        self.battery_percent = 100
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.ultra_distance = 0
        self.seat_detected = False
        self.current_caller = ""

        # 네비게이션(경로주행) 변수
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0
        self.waypoint_queue = []  # 경로 큐 (여기에 경유지들이 쌓임)

        # TCP 소켓 변수
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        self.backoff = 1.0        # 재연결 대기 시간 점증용
        self.next_connect_time = 0.0

        # 길찾기 객체 생성
        self.pathfinder = SimplePathFinder(self.map_file)

        # --- 2. ROS 2 통신 설정 (Subscriber & Publisher) ---
        prefix = f"/{self.robot_name}"
        
        # 구독 (센서 데이터 수신)
        self.create_subscription(Odometry, f"{prefix}/odom", self.odom_pose_cb, 10)
        self.create_subscription(BatteryState, f"{prefix}/battery_state", self.batt_cb, 10)
        self.create_subscription(Float32, f"{prefix}/ultra_distance_cm", self.ultra_cb, 10)
        self.create_subscription(Bool, f"{prefix}/seat_detected", self.seat_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/button", self.button_cb, 10)

        # 발행 (UI 및 네비게이션 제어)
        self.ui_pub = self.create_publisher(String, f"{prefix}/ui/info", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.caller_pub = self.create_publisher(String, f"{prefix}/caller_name", 10)

        # --- 3. 타이머 및 스레드 시작 ---
        # 0.5초(2Hz) 주기로 메인 제어 루프 실행
        self.create_timer(0.5, self.control_loop) 
        
        # TCP 수신은 블로킹 되므로 별도 스레드에서 실행
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        
        print("[Init] 모든 초기화 완료. 서버 연결 대기 중...")

    # =========================================================
    # 콜백 함수 (데이터 수신)
    # =========================================================
    def odom_pose_cb(self, msg):
        """ 오도메트리에서 로봇 위치(x,y)와 방향(theta) 업데이트 """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 쿼터니언 -> 오일러 각(Yaw) 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def batt_cb(self, msg):
        """ 배터리 상태 업데이트 """
        self.battery_percent = int(msg.percentage) if msg.percentage > 1.0 else int(msg.percentage * 100)

    def ultra_cb(self, msg):
        """ 초음파 거리 업데이트 """
        self.ultra_distance = int(msg.data)

    def seat_cb(self, msg):
        """ 착석 여부 업데이트 (True/False) """
        self.seat_detected = msg.data

    def button_cb(self, msg):
        """ 버튼 입력 처리 (0이 아닐 때만 반응) """
        btn = msg.data
        if btn != 0:
            print(f"\n[Button] 🔘 물리 버튼 입력 감지! 값: {btn}")
            self.handle_button_logic(btn)

    # =========================================================
    # 로직 (FSM 상태 머신 & 버튼 제어)
    # =========================================================
    def change_state(self, new_state):
        """ 로봇 상태 변경 및 로그 출력 """
        if self.current_state != new_state:
            print(f"[State] 🔄 상태 변경: {self.current_state} -> {new_state}")
            self.current_state = new_state
        # UI에 즉시 반영
        self.publish_ui_info()

    def handle_button_logic(self, btn):
        """ 버튼 값에 따른 로봇 행동 결정 (핵심 로직) """
        
        # 1. 탑승 대기 상태 -> [탑승 완료] 버튼 누름
        if self.current_state == STATE_BOARDING:
            if btn == BTN_BOARDING_COMPLETE: # 값: 1
                if self.seat_detected:
                    print("[Logic] ✅ 탑승 완료 & 착석 확인됨. 목적지로 출발합니다.")
                    self.change_state(STATE_RUNNING)
                    self.mission_mode = "DELIVER"
                    # 미리 저장된 최종 목적지로 주행 시작
                    self.start_path_navigation(self.final_goal_x, self.final_goal_y)
                else:
                    print("[Logic] ⚠️ 탑승 버튼 눌림. 그러나 FSR(압력) 미감지. (안전을 위해 출발 안함)")
                    # 테스트용으로 강제 출발하려면 아래 주석 해제
                    self.change_state(STATE_RUNNING)
                    self.mission_mode = "DELIVER"
                    self.start_path_navigation(self.final_goal_x, self.final_goal_y)

        # 2. 주행 중(이동, 픽업) -> [비상 정지] 버튼 누름
        elif self.current_state in [STATE_RUNNING, STATE_HEADING]:
            if btn == BTN_EMERGENCY: # 값: 4
                print("[Logic] 🚨 비상 정지 명령 수신! 제자리에 멈춥니다.")
                self.prev_state = self.current_state # 나중에 재개할 때 쓰려고 저장
                self.change_state(STATE_STOP)
                self.pause_nav2() # Nav2 정지 명령

        # 3. 비상 정지 상태 -> [주행 재개] 버튼 누름
        elif self.current_state == STATE_STOP:
            if btn == BTN_RESUME: # 값: 3
                print(f"[Logic] ▶️ 주행 재개. 이전 상태({self.prev_state})로 복귀합니다.")
                self.change_state(self.prev_state)
                # 멈췄던 지점(현재 목표)으로 다시 이동 명령
                print(f"[Nav] 목표 재전송: ({self.current_goal_x:.2f}, {self.current_goal_y:.2f})")
                self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

        # 4. 목적지 도착 상태 -> [하차 완료] 버튼 누름
        elif self.current_state == STATE_ARRIVED:
            if btn == BTN_EXIT_COMPLETE: # 값: 5
                if not self.seat_detected:
                    print("[Logic] ✅ 하차 완료 확인됨. 대기(Waiting) 모드로 복귀.")
                    self.reset_to_waiting()
                else:
                    print("[Logic] ⚠️ 하차 버튼 눌림. 그러나 아직 좌석에 사람이 감지됩니다.")
                    self.reset_to_waiting()

    def reset_to_waiting(self):
        """ 모든 미션 종료 후 초기화 """
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"
        self.current_caller = ""
        self.waypoint_queue = []
        self.publish_ui_info()
        print("[Logic] 로봇이 대기 상태로 초기화되었습니다.")

    # =========================================================
    # 네비게이션 제어 (Nav2)
    # =========================================================
    def publish_nav2_goal(self, x, y):
        """ ROS 2 Goal Topic 발행 (로봇을 이동시킴) """
        self.current_goal_x = x
        self.current_goal_y = y
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0 # 방향은 Nav2가 알아서 처리
        
        self.goal_pub.publish(goal)
        # (너무 자주 찍히면 주석 처리 가능)
        # print(f"[Nav] 이동 명령 전송 -> ({x:.2f}, {y:.2f})")

    def pause_nav2(self):
        """ 로봇을 현재 위치에 정지시킴 """
        # 현재 위치를 목표로 주면 멈춥니다.
        self.publish_nav2_goal(self.x, self.y)

    def start_path_navigation(self, tx, ty):
        """ A* 경로를 계산하고 첫 번째 경유지로 이동 시작 """
        path = self.pathfinder.get_path(self.x, self.y, tx, ty)
        print(f"[Nav] 경로 계획 완료: 총 {len(path)}개의 웨이포인트 생성됨.")
        self.waypoint_queue = path
        self.pop_and_drive()

    def pop_and_drive(self):
        """ 큐에서 다음 웨이포인트를 꺼내서 이동 """
        if self.waypoint_queue:
            wp = self.waypoint_queue.pop(0) # 큐의 맨 앞을 꺼냄
            print(f"[Nav] 📍 다음 경유지로 이동: ({wp[0]:.2f}, {wp[1]:.2f}) 남은 경로: {len(self.waypoint_queue)}개")
            self.publish_nav2_goal(wp[0], wp[1])
        else:
            print("[Nav] 모든 경유지 소진. 목표 지점 근처입니다.")

    # =========================================================
    # 메인 제어 루프 (타이머 호출 - 0.5초 주기)
    # =========================================================
    def control_loop(self):
        # 1. 서버 연결 관리
        if not self.connect(): return # 연결 안 되면 리턴
        
        # 2. 로그인 패킷 (최초 1회)
        self.send_login_once()
        
        # 3. UI 정보 주기적 갱신
        self.publish_ui_info()

        # 4. 주행 모니터링 (도착 판정)
        if self.current_state in [STATE_HEADING, STATE_RUNNING]:
            # 현재 목표까지 남은 거리 계산
            dist = math.dist((self.x, self.y), (self.current_goal_x, self.current_goal_y))
            
            # [중요] Look-ahead 로직
            # 경로가 남아있으면(중간 경유지) -> 1.2m 이내면 통과 (부드럽게)
            # 경로가 끝났으면(최종 목적지) -> 0.2m 이내여야 도착 (정밀하게)
            
            is_intermediate = (len(self.waypoint_queue) > 0)
            tolerance = DIST_TOLERANCE_WAYPOINT if is_intermediate else DIST_TOLERANCE_FINAL

            if dist < tolerance:
                if is_intermediate:
                    # 중간 경유지 도착 -> 멈추지 말고 다음 점으로!
                    self.pop_and_drive() 
                else:
                    # 진짜 최종 목적지 도착
                    print(f"[Nav] 🏁 최종 목표 지점 도착! (오차: {dist:.2f}m)")
                    self.handle_arrival()

        # 5. 서버로 로봇 상태 보고 (Heartbeat)
        self.send_robot_state()

    def handle_arrival(self):
        """ 목적지 도착 시 상태 전환 처리 """
        if self.current_state == STATE_HEADING:
            print("[Event] 환자 위치 도착 완료. 탑승 대기 상태로 전환.")
            self.change_state(STATE_BOARDING)
        
        elif self.current_state == STATE_RUNNING:
            print("[Event] 목적지 도착 완료.")
            self.change_state(STATE_ARRIVED)
            
            # 단순 이동(MOVE) 미션이었다면 사람을 내릴 필요 없으므로 바로 대기
            if self.mission_mode == "MOVE":
                print("[Event] 단순 이동 미션 종료 -> 대기 모드 자동 전환.")
                self.reset_to_waiting()

    def send_robot_state(self):
        """ 서버에 현재 상태 패킷 전송 """
        try:
            payload = struct.pack(STATE_FMT,
                int(self.battery_percent), self.x, self.y, self.theta,
                int(self.current_state),
                int(self.ultra_distance), int(1 if self.seat_detected else 0)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)
        except Exception as e:
            self.close_socket(f"TX Error: {e}")

    def publish_ui_info(self):
        """ STM32 UI 표시용 토픽 발행 (상태@호출자@출발@도착) """
        s_mode = str(self.current_state)
        s_caller = self.current_caller if self.current_caller else "Waiting"
        msg = f"{s_mode}@{s_caller}@-@-"
        self.ui_pub.publish(String(data=msg))

    # =========================================================
    # 네트워크 패킷 처리 (수신)
    # =========================================================
    def handle_server_message(self, msg_type, payload):
        """ 서버에서 온 메시지 해석 """
        if msg_type == MSG_ASSIGN_GOAL:
            # 목적지 할당 명령
            if len(payload) != GOAL_SIZE: return
            order, sx, sy, gx, gy, raw_name = struct.unpack(GOAL_FMT, payload)
            
            try: caller = raw_name.split(b'\x00')[0].decode('utf-8')
            except: caller = "Unknown"

            if order == 99: # 킬 스위치
                print("💀 [Server] Kill Command Received. Shutting down.")
                self.destroy_node(); sys.exit(0)

            print(f"\n[Server] 📩 명령 수신! OrderCode: {order}, Caller: {caller}")
            print(f"         목표 좌표: ({gx:.2f}, {gy:.2f})")
            
            self.current_caller = caller
            self.caller_pub.publish(String(data=caller))
            self.final_goal_x = gx
            self.final_goal_y = gy

            if order == 6: # 배차 명령 (환자 픽업)
                print("[Mission] 🚑 환자 픽업 미션 시작!")
                self.change_state(STATE_HEADING)
                self.mission_mode = "PICKUP"
                self.start_path_navigation(sx, sy) # 환자 위치로 이동
            
            elif order in [1, 4, 5]: # 단순 이동 (복귀/호출)
                print("[Mission] 🚌 단순 이동 미션 시작!")
                self.change_state(STATE_RUNNING)
                self.mission_mode = "MOVE"
                self.start_path_navigation(gx, gy) # 목적지로 이동

    # =========================================================
    # 소켓 기본 유틸리티
    # =========================================================
    def connect(self):
        """ 서버 연결 시도 """
        now = time.time()
        if now < self.next_connect_time: return False # 재연결 쿨타임 중
        
        with self.lock:
            if self.sock: return True # 이미 연결됨
            
            try:
                print(f"[Net] 서버 연결 시도... ({self.server_ip}:{self.server_port})")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None)
                
                self.sock = s
                self.logged_in = False
                self.backoff = 1.0
                print("[Net] ✅ 서버 연결 성공!")
                return True
            except Exception as e:
                # 연결 실패 시 점진적 대기 (Backoff)
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, 60.0)
                # print(f"[Net] 연결 실패 ({e}). {self.backoff}초 후 재시도.")
                return False

    def close_socket(self, reason):
        """ 소켓 종료 및 정리 """
        with self.lock:
            if self.sock: 
                try: self.sock.close()
                except: pass
            self.sock = None; self.logged_in = False
        print(f"[Net] 🔌 소켓 연결 종료: {reason}")

    def send_packet(self, msg_type, payload):
        """ 패킷 헤더 붙여서 전송 """
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        with self.lock:
            if not self.sock: return
            try: self.sock.sendall(header + payload)
            except Exception as e: self.close_socket(f"Send Err: {e}")

    def send_login_once(self):
        """ 최초 연결 시 로그인 패킷 1회 전송 """
        if self.logged_in: return
        print(f"[Net] 로그인 패킷 전송 (ID: {self.robot_name})")
        name_bytes = self.robot_name.encode("utf-8")[:64]
        self.send_packet(MSG_LOGIN_REQ, name_bytes)
        self.logged_in = True

    def recvall(self, sock, n):
        """ n바이트를 모두 받을 때까지 대기 """
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk: return b""
                data += chunk
            except: return b""
        return data

    def rx_loop(self):
        """ 수신 전담 스레드 """
        while self.running and rclpy.ok():
            with self.lock: sock = self.sock
            if sock is None:
                time.sleep(1.0); continue
            
            try:
                # 1. 헤더 수신
                hdr = self.recvall(sock, HDR_SIZE)
                if len(hdr) != HDR_SIZE:
                    self.close_socket("Header Size Error"); continue
                
                magic, dev, msg_type, length = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue
                
                # 2. 데이터 수신
                payload = self.recvall(sock, length) if length > 0 else b""
                if len(payload) != length:
                    self.close_socket("Payload Size Error"); continue
                
                # 3. 처리
                self.handle_server_message(msg_type, payload)
            except Exception as e:
                self.close_socket(f"RX Loop Error: {e}"); time.sleep(1.0)

# =========================================================
# 메인 실행부
# =========================================================
def main():
    rclpy.init()
    
    # 기본값 설정
    robot_name = "wc1"
    map_file = "map_graph.json"
    
    # 실행 인자로 로봇 이름 받기 (예: python3 tcp_bridge.py wc2 map.json)
    if len(sys.argv) > 1: robot_name = sys.argv[1]
    if len(sys.argv) > 2: map_file = sys.argv[2]
    
    node = TcpBridge(robot_name, map_file)
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        print("\n[Sys] 키보드 인터럽트 감지. 종료합니다.")
    finally:
        node.running = False
        node.close_socket("Shutdown")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()