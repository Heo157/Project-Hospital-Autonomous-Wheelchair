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
# PoseWithCovarianceStamped: AMCL 위치 정보 (공분산 포함)
# PoseStamped: Nav2 목표 지점 명령
# Odometry: 휠 인코더 기반 위치 정보
# BatteryState: 배터리 잔량
# Int32, Bool, String: 초음파 거리, 착석 여부, 호출자 이름 등 단순 데이터
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool, String

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


class TcpBridge(Node):
    """
    ROS 2 노드 클래스: TCP 통신과 ROS 토픽 중계 담당
    """
    def __init__(self, parameter_overrides=None):
        # 노드 이름 "tcp_bridge"로 초기화
        super().__init__("tcp_bridge", parameter_overrides=parameter_overrides)

        # ---------------------------------------------------------------------
        # 1. 파라미터 초기화 (외부에서 변경 가능)
        # ---------------------------------------------------------------------
        # 서버 IP와 포트 (기본값: 로컬호스트 8080)
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)
        
        # 로봇 이름 (예: wc1, wc2...) - C 서버가 실행 인자로 넘겨줌
        self.robot_name = self.declare_parameter("robot_name", "wc1").value
        
        # AMCL(지도 기반 위치) 사용 여부 (False면 Odom 사용)
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value)
        
        # 통신 주기 (Hz) - 초당 몇 번 서버로 데이터를 보낼지 (기본 2.0Hz = 0.5초)
        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)

        # [토픽 네임스페이스 설정]
        # 로봇 이름이 'wc1'이면 모든 토픽 앞에 '/wc1'이 붙습니다.
        prefix = f"/{self.robot_name}"
        
        # ---------------------------------------------------------------------
        # 2. ROS 2 토픽 이름 설정 (자동 생성)
        # ---------------------------------------------------------------------
        # [구독할 토픽들]
        self.topic_amcl = self.declare_parameter("topic_amcl", f"{prefix}/amcl_pose").value
        self.topic_odom = self.declare_parameter("topic_odom", f"{prefix}/odom").value
        self.topic_battery = self.declare_parameter("topic_battery", f"{prefix}/battery_state").value
        self.topic_ultra = self.declare_parameter("topic_ultra", f"{prefix}/ultra_distance_cm").value
        self.topic_seat = self.declare_parameter("topic_seat", f"{prefix}/seat_detected").value
        
        # [발행할 토픽들]
        self.topic_goal = self.declare_parameter("topic_goal", "/goal_pose").value # Nav2는 전역 토픽 사용
        self.topic_caller = self.declare_parameter("topic_caller", f"{prefix}/caller_name").value

        # ---------------------------------------------------------------------
        # 3. 내부 변수 초기화
        # ---------------------------------------------------------------------
        # 로봇 상태값
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery_percent = 90
        
        # 센서 데이터 (초음파 거리, 착석 여부)
        self.ultra_distance = 0
        self.seat_detected = False
        
        # 호출자 이름 (화면 표시용)
        self.current_caller = ""

        # FSM 상태 관리
        self.current_state = STATE_WAITING
        self.prev_state = STATE_WAITING
        self.mission_mode = "NONE" # NONE, PICKUP, DELIVER, CHARGE, DONE
        
        # 네비게이션 목표 좌표
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0

        # TCP 소켓 관련 변수
        self.sock = None
        self.lock = threading.Lock() # 스레드 간 충돌 방지
        self.logged_in = False
        self.running = True
        self.backoff = 1.0 # 재접속 대기 시간 (지수 백오프)
        self.next_connect_time = 0.0

        # ---------------------------------------------------------------------
        # 4. ROS 2 구독(Subscriber) 및 발행(Publisher) 설정
        # ---------------------------------------------------------------------
        
        # (1) 위치 정보 구독 (AMCL 사용 시 QoS 설정 필수)
        # AMCL은 'Transient Local' QoS를 사용하므로, 구독자도 맞춰줘야 데이터가 보임
        if self.use_amcl_pose:
            amcl_qos = QoSProfile(
                depth=1, 
                durability=DurabilityPolicy.VOLATILE, 
                reliability=ReliabilityPolicy.RELIABLE
            )
            self.create_subscription(PoseWithCovarianceStamped, self.topic_amcl, self.pose_cb, amcl_qos)
        else:
            self.create_subscription(Odometry, self.topic_odom, self.odom_pose_cb, 10)

        # (2) 센서 정보 구독
        self.create_subscription(BatteryState, self.topic_battery, self.batt_cb, 10)
        self.create_subscription(Int32, self.topic_ultra, self.ultra_cb, 10)
        self.create_subscription(Bool, self.topic_seat, self.seat_cb, 10)

        # (3) 정보 발행
        # Nav2에게 이동 명령 전달
        self.goal_pub = self.create_publisher(PoseStamped, self.topic_goal, 10)
        # 로봇 디스플레이(Qt/Web)에 호출자 이름 전달
        self.caller_pub = self.create_publisher(String, self.topic_caller, 10)

        # ---------------------------------------------------------------------
        # 5. 스레드 및 타이머 시작
        # ---------------------------------------------------------------------
        # (1) 데이터 전송 타이머 (주기적으로 C 서버에 상태 보고)
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)
        
        # (2) 데이터 수신 스레드 (C 서버에서 오는 명령을 상시 대기)
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"🚀 Bridge Started for [{self.robot_name}]")
        self.get_logger().info(f"📡 Topics: {self.topic_amcl}, {self.topic_ultra}, {self.topic_caller}")

    # =========================================================================
    # [섹션 A] ROS 콜백 함수 (데이터 수신)
    # =========================================================================
    
    def pose_cb(self, msg):
        """ AMCL 위치 데이터 수신 """
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_pose_cb(self, msg):
        """ Odom 위치 데이터 수신 (AMCL 안 쓸 때) """
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def batt_cb(self, msg):
        if msg.percentage is not None and msg.percentage >= 0.0:
            p = int(msg.percentage * 100.0) if msg.percentage <= 1.0 else int(msg.percentage)
            self.battery_percent = max(0, min(100, p))

    def ultra_cb(self, msg):
        """ 초음파 센서 데이터 수신 (Int32) """
        self.ultra_distance = msg.data

    def seat_cb(self, msg):
        """ 착석 감지 센서 데이터 수신 (Bool) """
        self.seat_detected = msg.data

    # =========================================================================
    # [섹션 B] 유틸리티 함수 (좌표 변환 및 상태 관리)
    # =========================================================================

    def get_state_name(self, state_id):
        """ 상태 ID를 문자열로 변환 (로그 출력용) """
        names = {
            STATE_WAITING: "WAITING", STATE_HEADING: "HEADING", STATE_BOARDING: "BOARDING",
            STATE_RUNNING: "RUNNING", STATE_STOP: "STOP", STATE_ARRIVED: "ARRIVED",
            STATE_EXITING: "EXITING", STATE_CHARGING: "CHARGING", STATE_ERROR: "ERROR"
        }
        return names.get(state_id, "UNKNOWN")

    def change_state(self, new_state):
        """ FSM 상태 변경 및 로그 출력 """
        if self.current_state != new_state:
            old = self.get_state_name(self.current_state)
            new = self.get_state_name(new_state)
            self.get_logger().info(f"[State Change] {old} -> {new}")
            self.current_state = new_state

    def quaternion_to_yaw(self, q):
        """ 쿼터니언(x,y,z,w) -> 오 Euler 각도(Yaw) 변환 """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """ Euler 각도(Yaw) -> 쿼터니언 변환 """
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.z = math.sin(yaw * 0.5)
        return q

    # =========================================================================
    # [섹션 C] Nav2 제어 및 시나리오 로직
    # =========================================================================

    def publish_nav2_goal(self, x, y):
        """ Nav2에게 목표 좌표 전송 """
        if self.current_state == STATE_STOP:
            self.get_logger().warn("⚠️ STOP 상태에서는 이동할 수 없습니다!")
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"  # 지도 좌표계 기준
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation = self.yaw_to_quaternion(0.0) # 방향은 일단 0도

        self.goal_pub.publish(goal)

        self.current_goal_x = float(x)
        self.current_goal_y = float(y)
        self.get_logger().info(f"Nav2 Goal -> ({x:.2f}, {y:.2f})")

    def stop_nav2(self):
        """ 로봇 정지 (현재 위치를 목표로 재설정) """
        self.publish_nav2_goal(self.x, self.y)

    def step_boarding_complete(self):
        """ [시나리오] 탑승 완료 후 목적지로 이동 """
        self.get_logger().info("✅ 탑승 완료. 목적지로 이동합니다.")
        self.change_state(STATE_RUNNING)
        self.mission_mode = "DELIVER"
        self.publish_nav2_goal(self.final_goal_x, self.final_goal_y)

    def step_exiting_complete(self):
        """ [시나리오] 하차 완료 후 대기 상태로 전환 """
        self.get_logger().info("✅ 하차 완료. 임무 종료.")
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"
        
        # 임무가 끝났으므로 호출자 이름 초기화
        self.current_caller = ""
        self.caller_pub.publish(String(data="Waiting..."))

    # =========================================================================
    # [섹션 D] 서버 메시지 처리 (수신부 핵심 로직)
    # =========================================================================

    def handle_server_message(self, msg_type, payload):
        """ C 서버로부터 받은 패킷을 분석하고 명령을 수행 """
        
        # [명령: 목표 할당 (배차, 이동, 충전 등)]
        if msg_type == MSG_ASSIGN_GOAL:
            
            # 1. 패킷 크기 검증 (84바이트여야 함)
            if len(payload) != GOAL_SIZE:
                self.get_logger().error(f"패킷 크기 불일치! 기대값: {GOAL_SIZE}, 실제값: {len(payload)}")
                return

            # 2. 구조체 언패킹 (명령, 시작좌표, 목표좌표, 호출자이름)
            order, sx, sy, gx, gy, raw_name = struct.unpack(GOAL_FMT, payload)

            # 3. 호출자 이름 디코딩 (C 문자열의 Null 바이트 제거)
            try:
                caller_name_str = raw_name.split(b'\x00')[0].decode('utf-8')
            except:
                caller_name_str = "Unknown"
            
            # 4. [자폭 기능] 서버가 Order 99를 보내면 프로세스 즉시 종료
            # (DB에서 로봇이 삭제되었을 때 발생)
            if order == 99:
                self.get_logger().fatal(f"💀 서버로부터 종료 명령(99)을 받았습니다.")
                self.get_logger().fatal(f"⛔ '{self.robot_name}' 로봇이 DB에 존재하지 않습니다. 프로세스를 종료합니다.")
                
                self.close_socket("Server Kill Command")
                self.destroy_node()
                sys.exit(0) # 프로그램 강제 종료

            self.get_logger().info(f"CMD 수신: Order={order}, Caller='{caller_name_str}'")

            # 5. 호출자 이름이 있으면 화면 표시용 토픽 발행
            if caller_name_str:
                self.current_caller = caller_name_str
                msg = String()
                msg.data = self.current_caller
                self.caller_pub.publish(msg)

            # 6. Order 번호에 따른 동작 수행
            if order == 6: # [배차 명령]
                # Start 지점으로 먼저 이동
                self.publish_nav2_goal(sx, sy)
                # 최종 목적지 저장
                self.final_goal_x = gx
                self.final_goal_y = gy
                self.change_state(STATE_HEADING)
                self.mission_mode = "PICKUP"

            elif order in [1, 4]: # [단순 이동 명령]
                self.publish_nav2_goal(gx, gy)
                self.change_state(STATE_RUNNING)
                self.mission_mode = "NONE"

            elif order == 5: # [충전소 이동 명령]
                self.publish_nav2_goal(gx, gy)
                self.change_state(STATE_RUNNING)
                self.mission_mode = "CHARGE"

            elif order == 2: # [비상 정지]
                if self.current_state != STATE_STOP:
                    self.prev_state = self.current_state
                    self.change_state(STATE_STOP)
                    self.stop_nav2()

            elif order == 3: # [동작 재개]
                if self.current_state == STATE_STOP:
                    self.get_logger().info("동작을 재개합니다...")
                    self.change_state(self.prev_state)
                    # 멈췄던 곳으로 다시 가거나, 원래 목표로 이동
                    self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

    # =========================================================================
    # [섹션 E] 데이터 전송 및 네트워크 관리 (송신부)
    # =========================================================================

    def tx_timer_cb(self):
        """ 주기적으로 실행되는 타이머 콜백 (상태 보고 및 도착 판정) """
        
        # 1. 서버 연결 확인 (연결 안 되어 있으면 재접속 시도)
        if not self.connect():
            return
        
        # 2. 로그인 패킷 전송 (최초 1회)
        self.send_login_once()

        try:
            # 3. 도착 판정 로직 (목표 지점과 현재 위치 거리 계산)
            dist = math.sqrt((self.x - self.current_goal_x)**2 + (self.y - self.current_goal_y)**2)
            
            # (A) Start 지점 도착 -> 탑승 대기 (5초)
            if self.current_state == STATE_HEADING and dist < 0.2:
                self.get_logger().info("🚩 출발지 도착. 탑승 대기 중 (5초)...")
                self.change_state(STATE_BOARDING)
                threading.Timer(5.0, self.step_boarding_complete).start()

            # (B) Goal 지점 도착
            elif self.current_state == STATE_RUNNING and dist < 0.2:
                self.get_logger().info("🚩 목적지 도착.")
                self.change_state(STATE_ARRIVED)

                if self.mission_mode == "DELIVER": # 환자 이송 완료
                    self.get_logger().info("하차 대기 중 (5초)...")
                    self.change_state(STATE_EXITING)
                    threading.Timer(5.0, self.step_exiting_complete).start()
                    self.mission_mode = "DONE_WAIT"

                elif self.mission_mode == "CHARGE": # 충전소 도착
                    self.change_state(STATE_CHARGING)
                    self.mission_mode = "DONE_CHARGE"

                elif self.mission_mode == "NONE": # 단순 이동 완료
                    self.change_state(STATE_WAITING)

            # 4. 호출자 이름 주기적 발행 (UI 갱신용 안전장치)
            if self.current_caller:
                msg = String()
                msg.data = self.current_caller
                self.caller_pub.publish(msg)

            # 5. [서버 전송] 로봇 상태 패킷 생성 (22바이트)
            # 구조: 배터리, X, Y, Theta, 상태, 초음파, 착석여부
            payload = struct.pack(
                STATE_FMT,
                int(self.battery_percent),
                float(self.x), float(self.y), float(self.theta),
                int(self.current_state),
                int(self.ultra_distance),           # 초음파 거리 (cm)
                int(1 if self.seat_detected else 0) # 착석 여부 (1/0)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            self.get_logger().error(f"데이터 전송 실패: {e}")
            self.close_socket("TX Error")

    # -------------------------------------------------------------------------
    # 소켓 유틸리티 함수들
    # -------------------------------------------------------------------------
    
    def _set_keepalive(self, s):
        """ TCP Keepalive 설정 (연결 끊김 조기 감지) """
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except:
            pass

    def connect(self):
        """ 서버에 연결 시도 (재접속 쿨타임 적용) """
        now = time.time()
        if now < self.next_connect_time: return False # 쿨타임 중
        
        with self.lock:
            if self.sock: return True # 이미 연결됨
            
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0) # 연결 타임아웃 3초
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None) # 연결 후엔 블로킹 모드
                
                self.sock = s
                self.logged_in = False
                self.backoff = 1.0 # 백오프 초기화
                self.next_connect_time = 0.0
                return True
            except:
                if self.sock: s.close()
                self.sock = None
                self.logged_in = False
                # 연결 실패 시 지수 백오프 적용 (1초 -> 2초 -> 4초 ... 최대 60초)
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, 60.0)
                return False

    def close_socket(self, reason):
        """ 소켓 종료 및 정리 """
        with self.lock:
            if self.sock:
                try: self.sock.close()
                except: pass
            self.sock = None
            self.logged_in = False
        self.get_logger().warn(f"소켓 연결 종료됨: {reason}")

    def send_packet(self, msg_type, payload):
        """ 패킷 생성 및 전송 (헤더 + 페이로드) """
        if len(payload) > 255: return # 페이로드 길이 제한
        
        # 헤더 패킹
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        
        with self.lock:
            if not self.sock: return
            try:
                self.sock.sendall(header + payload)
            except Exception as e:
                self.close_socket(f"Send Error: {e}")

    def send_login_once(self):
        """ 로그인 요청 패킷 전송 (연결 직후 1회) """
        if self.logged_in: return
        # 로봇 이름을 페이로드로 전송
        self.send_packet(MSG_LOGIN_REQ, self.robot_name.encode("utf-8")[:64])
        self.logged_in = True
        self.get_logger().info(f"로그인 요청 전송: {self.robot_name}")

    def recvall(self, sock, n):
        """ 지정된 n바이트만큼 확실히 읽어오는 함수 """
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk: return b"" # 연결 끊김
                data += chunk
            except: return b""
        return data

    def rx_loop(self):
        """ 데이터 수신 스레드 루프 """
        while self.running and rclpy.ok():
            with self.lock: sock = self.sock
            
            # 연결 없으면 대기
            if sock is None:
                time.sleep(1.0)
                continue
                
            try:
                # 1. 헤더 읽기 (4바이트)
                hdr = self.recvall(sock, HDR_SIZE)
                if len(hdr) != HDR_SIZE:
                    self.close_socket("Header Error")
                    continue
                
                magic, dev, msg_type, length = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue # 매직넘버 불일치 무시

                # 2. 페이로드 읽기 (가변 길이)
                payload = b""
                if length > 0:
                    payload = self.recvall(sock, length)
                    if len(payload) != length:
                        self.close_socket("Payload Error")
                        continue
                
                # 3. 메시지 처리 핸들러 호출
                self.handle_server_message(msg_type, payload)
                
            except Exception as e:
                self.close_socket(f"RX Error: {e}")
                time.sleep(1.0)


def main():
    """ 메인 함수: 노드 실행 및 종료 처리 """
    rclpy.init()
    
    # C 서버(Robot Manager)에서 실행 시 argv[1]로 로봇 이름(wc1 등)을 넘겨줌
    robot_name = "wc1" # 기본값
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

    # 노드 생성 및 실행
    node = TcpBridge(parameter_overrides=[Parameter("robot_name", Parameter.Type.STRING, robot_name)])

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