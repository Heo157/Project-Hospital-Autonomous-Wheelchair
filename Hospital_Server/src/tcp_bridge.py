"""
============================================================================
 파일명: tcp_bridge.py
 설명:   ROS 2(Robot) <-> TCP(C Server) 간의 통신 중계 및 FSM 시나리오 제어기
 작성일: 2025-12-27 (업데이트: FSM Full Implementation)
 
 [기능 요약]
 1. C 서버로부터 TCP 연결을 맺고 명령(Order)을 수신합니다.
 2. 수신된 명령(배차, 충전, 대기 등)에 따라 로봇의 상태(State)를 변경합니다.
 3. Nav2(자율주행)에게 목표 좌표를 발행하여 로봇을 이동시킵니다.
 4. 로봇의 위치와 상태를 주기적으로 체크하여 시나리오(탑승, 하차 등)를 진행합니다.
 5. 로봇의 현재 상태(위치, 배터리, 동작상태)를 서버에 실시간으로 보고합니다.

 [FSM 상태 정의 (명세서 반영)]
 - WAITING:  대기 중 (임무 없음)
 - HEADING:  [배차 1단계] 환자가 있는 호출지(Start)로 이동 중
 - BOARDING: [배차 2단계] 호출지 도착 후 승객 탑승 대기 (5초간 정지)
 - RUNNING:  목적지(Goal)로 이동 중 (일반 이동 또는 배차 주행)
 - STOP:     이동 중 관리자 명령에 의한 일시 정지
 - ARRIVED:  목적지 도착 (잠시 거쳐가는 상태)
 - EXITING:  [배차 3단계] 목적지 도착 후 승객 하차 대기 (5초간 정지)
 - CHARGING: [충전] 충전소 도착 후 충전 진행 중
 - ERROR:    에러 발생
============================================================================
"""

import sys
import socket
import struct
import threading
import time
import math

# ROS 2 관련 라이브러리
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# 메시지 타입: 위치(Pose), 방향(Quaternion), 오도메트리(Odometry), 배터리(BatteryState)
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# ==========================================
# 1. 프로토콜 & 상수 정의
# (C언어 서버의 헤더 파일과 반드시 일치해야 통신 가능)
# ==========================================

# 패킷 유효성 검사를 위한 매직 넘버 (0xAB) - 이 값이 안 맞으면 패킷 버림
MAGIC_NUMBER = 0xAB 
# 장치 ID (이 코드는 로봇 역할을 하므로 0x02)
DEVICE_ROBOT_ROS = 0x02

# [메시지 타입 정의]
MSG_LOGIN_REQ   = 0x01  # 로그인 요청 (접속하자마자 이름 보낼 때 사용)
MSG_ROBOT_STATE = 0x20  # 상태 보고 (로봇 -> 서버: 내 위치랑 상태는 이래요)
MSG_ASSIGN_GOAL = 0x30  # 목표 할당 (서버 -> 로봇: 여기로 가세요)

# [FSM 상태 상수]
# 로봇의 현재 행동을 나타내는 숫자값 (DB의 status 컬럼과 매핑됨)
STATE_WAITING  = 0  # 대기
STATE_HEADING  = 1  # 픽업 이동
STATE_BOARDING = 2  # 탑승 중
STATE_RUNNING  = 3  # 주행 중
STATE_STOP     = 4  # 정지
STATE_ARRIVED  = 5  # 도착
STATE_EXITING  = 6  # 하차 중
STATE_CHARGING = 7  # 충전 중
STATE_ERROR    = 99 # 에러

# [바이너리 데이터 포맷 정의 (struct)]
# <: 리틀 엔디안 (Intel/ARM CPU 표준)
# B: unsigned char(1byte), f: float(4bytes), i: int(4bytes)

# 1. 헤더 포맷 (총 4바이트)
# 구조: magic(1) + device(1) + msg_type(1) + payload_len(1)
HDR_FMT = "<BBBB"      
HDR_SIZE = struct.calcsize(HDR_FMT)

# 2. 상태 보고 포맷 (총 17바이트)
# 구조: battery(int) + x(float) + y(float) + theta(float) + state(unsigned char)
STATE_FMT = "<ifffB"   
STATE_SIZE = struct.calcsize(STATE_FMT)

# 3. 목표 할당 포맷 (총 20바이트)
# 구조: order(int) + start_x(float) + start_y(float) + goal_x(float) + goal_y(float)
# 설명: 일반 이동일 때는 start 좌표를 무시하지만, 배차(Order 6)일 때는 start와 goal을 모두 사용함
GOAL_FMT = "<iffff"    
GOAL_SIZE = struct.calcsize(GOAL_FMT)


class TcpBridge(Node):
    """
    ROS 2 노드이자 TCP 클라이언트 역할을 동시에 수행하는 클래스
    """
    def __init__(self, parameter_overrides=None):
        # ROS 2 노드 초기화 (노드 이름: tcp_bridge)
        super().__init__("tcp_bridge", parameter_overrides=parameter_overrides)

        # ------------------------------------------
        # 1. 파라미터 & 변수 초기화
        # ------------------------------------------
        # 서버 접속 정보 (launch 파일이나 명령행 인자에서 변경 가능)
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)
        
        # 로봇 이름 (예: wc1, wc2). 이 이름이 토픽의 네임스페이스가 됨
        self.robot_name = self.declare_parameter("robot_name", "wc1").value
        
        # 위치 추정 방식 선택 (True: AMCL 사용 / False: 오도메트리만 사용)
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value)
        
        # 상태 전송 주기 (Hz). 2.0이면 초당 2번 서버로 상태를 보냄
        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)
        
        # Nav2 목표 토픽 이름
        self.goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        
        # 토픽 접두어 (예: /wc1)
        self.topic_prefix = f"/{self.robot_name}"
        
        # [로봇 상태 변수] - 센서로부터 계속 업데이트됨
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery_percent = 90
        
        # [핵심] FSM 상태 관리 변수
        self.current_state = STATE_WAITING  # 현재 상태
        self.prev_state = STATE_WAITING     # '정지(STOP)' 해제 시 원래 상태로 복귀하기 위한 저장소
        self.mission_mode = "NONE"          # 현재 수행 중인 미션 종류 ("NONE", "PICKUP", "DELIVER", "CHARGE")

        # [목표 지점 관리 변수]
        # current: Nav2가 지금 당장 가고 있는 좌표 (도착 확인용 거리 계산에 사용)
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        # final: 배차 시나리오에서 환자를 태운 뒤 가야 할 최종 목적지 (메모리에 저장해둠)
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0

        # [네트워크 관련 변수]
        self.sock = None
        self.lock = threading.Lock() # 스레드 간 충돌 방지 (송신/수신 동시 접근 막기)
        self.logged_in = False       # 로그인 패킷 전송 여부
        self.running = True          # 프로그램 종료 플래그
        self.backoff = 1.0           # 재접속 대기 시간 (실패할수록 늘어남)
        self.next_connect_time = 0.0 # 다음 접속 시도 가능 시각

        # ------------------------------------------
        # 2. ROS 2 통신 설정 (Subscriber & Publisher)
        # ------------------------------------------
        # (1) 위치 정보 구독
        if self.use_amcl_pose:
            # AMCL(지도 기반 위치 추정) 사용 시
            self.create_subscription(PoseWithCovarianceStamped, f"{self.topic_prefix}/amcl_pose", self.pose_cb, 10)
        else:
            # 오도메트리(바퀴 회전 기반) 사용 시
            self.create_subscription(Odometry, f"{self.topic_prefix}/odom", self.odom_pose_cb, 10)

        # (2) 배터리 정보 구독
        self.create_subscription(BatteryState, f"{self.topic_prefix}/battery_state", self.batt_cb, 10)
        
        # (3) 목표 지점 발행 (여기로 메시지를 쏘면 Nav2가 로봇을 움직임)
        self.goal_pub = self.create_publisher(PoseStamped, f"{self.topic_prefix}/{self.goal_topic}", 10)

        # ------------------------------------------
        # 3. 타이머 & 스레드 시작
        # ------------------------------------------
        # 메인 타이머: 주기적으로 상태를 체크하고 서버로 데이터를 보냄
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)
        
        # 수신 스레드: 서버에서 오는 데이터를 기다림 (Blocking 되므로 별도 스레드 사용)
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"FSM Bridge Started for [{self.robot_name}]")

    # =========================================================
    # [Helper] 유틸리티 함수 (로그 및 계산용)
    # =========================================================
    def get_state_name(self, state_id):
        """숫자로 된 상태 ID를 사람이 읽기 쉬운 문자열로 변환"""
        names = {
            STATE_WAITING: "WAITING", STATE_HEADING: "HEADING", STATE_BOARDING: "BOARDING",
            STATE_RUNNING: "RUNNING", STATE_STOP: "STOP", STATE_ARRIVED: "ARRIVED",
            STATE_EXITING: "EXITING", STATE_CHARGING: "CHARGING", STATE_ERROR: "ERROR"
        }
        return names.get(state_id, "UNKNOWN")

    def change_state(self, new_state):
        """로봇의 상태를 변경하고 로그를 출력"""
        if self.current_state != new_state:
            old = self.get_state_name(self.current_state)
            new = self.get_state_name(new_state)
            self.get_logger().info(f"[State Change] {old} -> {new}")
            self.current_state = new_state

    def quaternion_to_yaw(self, q):
        """ROS의 쿼터니언(x,y,z,w) 방향을 2D 각도(Yaw)로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """2D 각도(Yaw)를 ROS 쿼터니언 메시지로 변환"""
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.z = math.sin(yaw * 0.5)
        return q

    # =========================================================
    # [Nav2 Control] 주행 명령 관련 함수
    # =========================================================
    def publish_nav2_goal(self, x, y):
        """
        Nav2에게 '이 좌표로 이동해!'라고 명령을 내리는 함수
        """
        # 비상 정지(STOP) 상태라면 명령을 무시함 (안전 장치)
        if self.current_state == STATE_STOP:
            self.get_logger().warn("Cannot move in STOP state!")
            return

        # ROS 2 메시지 생성
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map" # 지도 좌표계 기준
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation = self.yaw_to_quaternion(0.0) # 방향은 기본 0도
        
        # 토픽 발행 (Nav2가 수신)
        self.goal_pub.publish(goal)
        
        # 현재 목표 업데이트 (나중에 도착했는지 거리를 재기 위함)
        self.current_goal_x = float(x)
        self.current_goal_y = float(y)
        self.get_logger().info(f"Nav2 Goal -> ({x:.2f}, {y:.2f})")

    def stop_nav2(self):
        """
        로봇을 즉시 정지시키는 함수.
        원리: '지금 있는 바로 그 자리'를 목표로 다시 줘서 멈추게 함.
        """
        self.publish_nav2_goal(self.x, self.y)

    # =========================================================
    # [Scenario] 시나리오 단계별 지연 처리 (Callback)
    # =========================================================
    def step_boarding_complete(self):
        """
        [시나리오] 환자 탑승 대기(5초)가 끝난 후 호출되는 함수
        행동: 상태를 RUNNING으로 바꾸고, 메모리에 저장해둔 '최종 목적지'로 출발
        """
        self.get_logger().info("✅ Passenger Boarded. Heading to Destination.")
        self.change_state(STATE_RUNNING)   # 상태: 주행 중
        self.mission_mode = "DELIVER"      # 모드: 환자 이송
        # 최종 목적지(Goal)로 이동 명령 발행
        self.publish_nav2_goal(self.final_goal_x, self.final_goal_y)

    def step_exiting_complete(self):
        """
        [시나리오] 환자 하차 대기(5초)가 끝난 후 호출되는 함수
        행동: 모든 임무를 마치고 대기(WAITING) 상태로 복귀
        """
        self.get_logger().info("✅ Passenger Exited. Mission Complete.")
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"

    # =========================================================
    # [Logic] 서버 메시지 처리 핸들러 (Order 분기 처리)
    # =========================================================
    def handle_server_message(self, msg_type, payload):
        """서버로부터 받은 패킷을 분석하여 로봇을 제어하는 핵심 함수"""
        
        # 메시지 타입: 목표 할당 (Goal Assignment)
        if msg_type == MSG_ASSIGN_GOAL:
            # 데이터 길이 체크
            if len(payload) != GOAL_SIZE: return
            
            # 구조체 언패킹 (order, start_x, start_y, goal_x, goal_y)
            order, sx, sy, gx, gy = struct.unpack(GOAL_FMT, payload)
            
            self.get_logger().info(f"CMD Order={order} | Start({sx:.1f},{sy:.1f}) Goal({gx:.1f},{gy:.1f})")

            # --------------------------------------------------
            # [Order 1, 4] 일반 이동 명령 (관리자 호출, 대기 복귀)
            # --------------------------------------------------
            if order in [1, 4]:
                self.publish_nav2_goal(gx, gy)  # 바로 목적지로 이동
                self.change_state(STATE_RUNNING)
                self.mission_mode = "NONE"

            # --------------------------------------------------
            # [Order 5] 충전소 이동 명령
            # --------------------------------------------------
            elif order == 5:
                self.publish_nav2_goal(gx, gy)
                self.change_state(STATE_RUNNING)
                self.mission_mode = "CHARGE" # 도착하면 '충전 중' 상태로 바꾸기 위해 모드 설정

            # --------------------------------------------------
            # [Order 6] 배차 명령 (가장 복잡한 시나리오)
            # 순서: Start이동 -> 탑승(5초) -> Goal이동 -> 하차(5초)
            # --------------------------------------------------
            elif order == 6:
                # 1. 우선 환자가 있는 '출발지(Start)'로 이동
                self.publish_nav2_goal(sx, sy)
                
                # 2. 환자를 태우고 갈 '최종 목적지(Goal)'는 메모리에 기억해둠
                self.final_goal_x = gx
                self.final_goal_y = gy
                
                # 3. 상태 변경: 픽업하러 가는 중
                self.change_state(STATE_HEADING) 
                self.mission_mode = "PICKUP"

            # --------------------------------------------------
            # [Order 2] 비상 정지 (STOP)
            # --------------------------------------------------
            elif order == 2:
                if self.current_state != STATE_STOP:
                    self.prev_state = self.current_state # 현재 상태를 기억해둠 (재개할 때 쓰려고)
                    self.change_state(STATE_STOP)        # 상태: 정지
                    self.stop_nav2()                     # 로봇을 제자리에 멈춤

            # --------------------------------------------------
            # [Order 3] 동작 재개 (RESUME)
            # --------------------------------------------------
            elif order == 3:
                if self.current_state == STATE_STOP:
                    self.get_logger().info("Resuming operation...")
                    # 1. 이전 상태로 복귀 (예: RUNNING이었다면 다시 RUNNING으로)
                    self.change_state(self.prev_state)
                    # 2. 멈추기 전 가려던 목표로 다시 이동 명령
                    self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

    # =========================================================
    # [Timer] 주기적 상태 체크 & FSM 자동 전이 로직
    # =========================================================
    def tx_timer_cb(self):
        """
        이 함수는 지정된 주기(예: 0.5초)마다 자동으로 실행됩니다.
        1. 서버 연결 확인
        2. 로봇이 목표에 도착했는지 거리 계산
        3. 시나리오에 따른 상태 자동 변경 (이동 -> 도착 -> 대기 등)
        4. 현재 상태를 서버로 보고
        """
        # 서버 연결 시도 (끊겼으면 재접속)
        if not self.connect(): return
        
        # 최초 1회 로그인 패킷 전송
        self.send_login_once()

        try:
            # 현재 위치와 목표 위치 사이의 거리 계산 (유클리드 거리)
            dist = math.sqrt((self.x - self.current_goal_x)**2 + (self.y - self.current_goal_y)**2)
            
            # -----------------------------------------------------------------
            # [FSM 전이 1] 픽업 이동 중(HEADING) -> 출발지 도착 -> 탑승 대기(BOARDING)
            # -----------------------------------------------------------------
            # 조건: 상태가 HEADING이고, 목표까지 거리가 0.5m 이내일 때
            if self.current_state == STATE_HEADING and dist < 0.5:
                self.get_logger().info("🚩 Arrived at Start. Boarding (Wait 5s)...")
                self.change_state(STATE_BOARDING)
                
                # 5초 뒤에 자동으로 'step_boarding_complete' 함수 실행 (비동기 타이머)
                threading.Timer(5.0, self.step_boarding_complete).start()

            # -----------------------------------------------------------------
            # [FSM 전이 2] 주행 중(RUNNING) -> 목적지 도착 -> 다음 행동 분기
            # -----------------------------------------------------------------
            # 조건: 상태가 RUNNING이고, 목표까지 거리가 0.5m 이내일 때
            elif self.current_state == STATE_RUNNING and dist < 0.5:
                self.get_logger().info("🚩 Arrived at Destination.")
                self.change_state(STATE_ARRIVED) # 일단 '도착' 상태로 변경

                # 미션 모드에 따라 다음 행동 결정
                if self.mission_mode == "DELIVER":
                    # (배차 미션) 목적지 도착 -> 하차 대기(EXITING) 시작
                    self.get_logger().info("State: ARRIVED -> EXITING (Wait 5s)...")
                    self.change_state(STATE_EXITING)
                    # 5초 뒤에 임무 종료 함수 실행
                    threading.Timer(5.0, self.step_exiting_complete).start()
                    self.mission_mode = "DONE_WAIT" # 중복 실행 방지용 모드 변경
                
                elif self.mission_mode == "CHARGE":
                    # (충전 미션) 충전소 도착 -> 충전 중(CHARGING) 상태로 변경
                    self.get_logger().info("State: ARRIVED -> CHARGING")
                    self.change_state(STATE_CHARGING)
                    self.mission_mode = "DONE_CHARGE"

                elif self.mission_mode == "NONE":
                    # (일반 이동) 도착 -> 대기(WAITING) 상태로 복귀
                    self.get_logger().info("State: ARRIVED -> WAITING")
                    self.change_state(STATE_WAITING)

            # -----------------------------------------------------------------
            # [서버 보고] 현재 로봇의 상태 패킷 전송
            # -----------------------------------------------------------------
            # C언어 구조체 포맷(STATE_FMT)에 맞춰 바이너리 데이터 생성
            payload = struct.pack(STATE_FMT, 
                                  int(self.battery_percent),        # 배터리
                                  float(self.x), float(self.y), float(self.theta), # 위치/자세
                                  int(self.current_state))          # 현재 상태 코드
            
            # 소켓으로 전송
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            self.get_logger().error(f"TX Fail: {e}")
            self.close_socket("TX Error")

    # =========================================================
    # [Network] 소켓 통신 기본 함수들 (Boilerplate)
    # =========================================================
    def _set_keepalive(self, s):
        """연결 끊김을 감지하기 위한 Keepalive 설정"""
        try: s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except: pass

    def connect(self):
        """서버에 접속을 시도하는 함수 (재접속 대기 로직 포함)"""
        now = time.time()
        # 재접속 대기 시간 중이면 시도하지 않음
        if now < self.next_connect_time: return False
        
        with self.lock:
            if self.sock: return True # 이미 연결되어 있음

            try:
                # 소켓 생성 및 접속
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0) # 3초 타임아웃
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None) # 접속 후에는 블로킹 모드
                
                self.sock = s
                self.logged_in = False
                self.backoff = 1.0 # 접속 성공 시 대기 시간 초기화
                self.next_connect_time = 0.0
                return True
            except:
                # 접속 실패 시 정리
                if self.sock: s.close()
                self.sock = None
                self.logged_in = False
                # 다음 접속 시도 시간 설정 (지수 백오프: 실패할수록 대기 시간 2배 증가)
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, self.backoff_max)
                return False

    def close_socket(self, reason):
        """소켓을 안전하게 닫는 함수"""
        with self.lock:
            if self.sock:
                try: self.sock.close()
                except: pass
            self.sock = None
            self.logged_in = False
        self.get_logger().warn(f"Socket closed: {reason}")

    def send_packet(self, msg_type, payload):
        """패킷을 조립(헤더+데이터)하여 전송하는 함수"""
        if len(payload) > 255: return # 페이로드 크기 제한
        # 헤더 생성: Magic + DeviceID + Type + Length
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        
        with self.lock:
            if not self.sock: return
            try: self.sock.sendall(header + payload)
            except Exception as e: self.close_socket(f"Send Error: {e}")

    def send_login_once(self):
        """최초 1회 로봇 이름을 전송하여 로그인"""
        if self.logged_in: return
        self.send_packet(MSG_LOGIN_REQ, self.robot_name.encode("utf-8")[:64])
        self.logged_in = True
        self.get_logger().info(f"Sent LOGIN_REQ: {self.robot_name}")

    def recvall(self, sock, n):
        """지정된 바이트 수(n)만큼 데이터를 확실하게 읽어오는 함수"""
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk: return b"" # 연결 끊김
                data += chunk
            except: return b""
        return data

    def rx_loop(self):
        """서버 데이터를 수신하는 별도 스레드 함수"""
        while self.running and rclpy.ok():
            with self.lock: sock = self.sock
            if sock is None:
                time.sleep(1.0)
                continue
            try:
                # 1. 헤더 읽기 (4바이트)
                hdr = self.recvall(sock, HDR_SIZE)
                if len(hdr) != HDR_SIZE:
                    self.close_socket("Header Error")
                    continue
                
                # 헤더 파싱
                magic, dev, msg_type, length = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue # 잘못된 패킷 무시

                # 2. 페이로드(데이터) 읽기
                payload = b""
                if length > 0:
                    payload = self.recvall(sock, length)
                    if len(payload) != length:
                        self.close_socket("Payload Error")
                        continue
                
                # 3. 메시지 처리 함수 호출
                self.handle_server_message(msg_type, payload)
                
            except Exception as e:
                self.close_socket(f"RX Error: {e}")
                time.sleep(1.0)

    # =========================================================
    # [ROS Callbacks] 토픽 구독 콜백 함수들
    # =========================================================
    def pose_cb(self, msg):
        """AMCL 위치 정보 업데이트"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
    def odom_pose_cb(self, msg):
        """오도메트리 위치 정보 업데이트"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
    def batt_cb(self, msg):
        """배터리 상태 업데이트"""
        if msg.percentage is not None:
            self.battery_percent = int(msg.percentage * 100)

def main():
    """프로그램 진입점"""
    rclpy.init()
    
    # 기본 로봇 이름 설정
    robot_name = "wc1"
    # 실행 인자가 있으면 덮어씌움 (예: python3 tcp_bridge.py wc2)
    if len(sys.argv) > 1: robot_name = sys.argv[1]
    
    # 노드 생성 및 파라미터 주입
    node = TcpBridge(parameter_overrides=[Parameter("robot_name", Parameter.Type.STRING, robot_name)])
    
    try: 
        rclpy.spin(node) # 노드 실행 (무한 루프)
    except KeyboardInterrupt: 
        pass
    finally:
        # 종료 처리
        node.running = False
        node.close_socket("Shutdown")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()