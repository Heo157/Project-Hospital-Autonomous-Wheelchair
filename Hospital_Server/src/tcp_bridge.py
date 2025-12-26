"""
============================================================================
 파일명: tcp_bridge.py
 설명:   ROS 2(Robot) <-> TCP(C Server) 간의 통신 중계 브리지
 작성일: 2025-12-04
 
 [핵심 기능]
 1. C 서버가 fork() & exec()로 이 스크립트를 실행합니다.
 2. 시스템 인자(sys.argv)로 전달받은 '로봇 이름'으로 ROS 2 노드를 생성합니다.
 3. ROS 2 토픽(Odom, Battery)을 구독하여 로봇 상태를 파악합니다.
 4. 주기적으로 C 서버(Localhost)에 TCP로 접속하여 상태 패킷을 전송합니다.
 5. C 서버로부터 목표 지점(Goal) 패킷이 오면 파싱하여 ROS 2(Nav2)로 발행합니다.
============================================================================
"""

import sys
import socket
import struct
import threading
import time
import math

# ROS 2 라이브러리 임포트
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# ==========================================
# 1. 프로토콜 정의 (common_defs.h와 100% 일치 필수)
# ==========================================

# 패킷 유효성 검사를 위한 매직 넘버 (0xAB)
MAGIC_NUMBER = 0xAB 
# 장치 ID (이 코드는 로봇 역할을 하므로 0x02)
DEVICE_ROBOT_ROS = 0x02

# 메시지 타입 정의 (명령 및 보고 종류)
MSG_LOGIN_REQ   = 0x01  # 로그인 요청 (내 이름 알림)
MSG_ROBOT_STATE = 0x20  # 상태 보고 (배터리, 위치 등)
MSG_ASSIGN_GOAL = 0x30  # 목표 지점 할당 (서버 -> 로봇)

# 로봇 상태 상수 (C 코드의 enum과 일치시켜야 함)
STATE_WAITING  = 0  # 대기 중
STATE_HEADING  = 1  # 이동 중
STATE_BOARDING = 2  # 탑승 중
STATE_RUNNING  = 3  # 주행 중
STATE_STOP     = 4  # 정지
STATE_ARRIVED  = 5  # 도착
STATE_EXITING  = 6  # 하차 중
STATE_CHARGING = 7  # 충전 중
STATE_ERROR    = 99 # 에러 발생

# ---------------------------------------------------------
# [중요] C언어 구조체 바이너리 포맷 정의
# struct.pack/unpack에 사용되는 포맷 문자열입니다.
# < : 리틀 엔디안 (Intel/AMD CPU 표준)
# B : unsigned char (1 byte)
# H : unsigned short (2 bytes)
# I : unsigned int (4 bytes)
# i : int (4 bytes)
# f : float (4 bytes)
# ---------------------------------------------------------

# 헤더 포맷: PacketHeader 구조체 대응
# 구조: magic(1) + device(1) + msg_type(1) + payload_len(1) = 총 4바이트
HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)

# 상태 데이터 포맷: RobotStateData 구조체 대응 (__attribute__((packed)))
# 구조: battery(4) + x(4) + y(4) + theta(4) + state(1) = 총 17바이트
STATE_FMT = "<ifffB"
STATE_SIZE = struct.calcsize(STATE_FMT)

# 목표 데이터 포맷: GoalAssignData 구조체 대응
# 구조: x(4) + y(4) = 총 8바이트
GOAL_FMT = "<ff"
GOAL_SIZE = struct.calcsize(GOAL_FMT)


class TcpBridge(Node):
    """
    ROS 2 노드이자 TCP 클라이언트 역할을 하는 클래스
    """
    def __init__(self, parameter_overrides=None):
        # 상위 클래스(Node) 초기화
        # parameter_overrides: main 함수에서 넘겨준 파라미터(로봇 이름)를 적용하기 위함
        super().__init__("tcp_bridge", parameter_overrides=parameter_overrides)

        # ------------------------------------------
        # 2. 파라미터 설정 및 초기화
        # ------------------------------------------
        
        # [네트워크 설정]
        # C 서버가 부모 프로세스이므로, 무조건 Localhost(127.0.0.1)로 접속합니다.
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)
        
        # [로봇 설정]
        # C 서버가 실행 인자로 넘겨준 로봇 이름 (예: "wc1", "tb3")
        self.robot_name = self.declare_parameter("robot_name", "wc1").value
        
        # 위치 추정 소스 선택 (True: AMCL 사용, False: 오도메트리 사용)
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value)
        
        # 서버 전송 주기 (Hz) - 예: 2.0이면 초당 2회 전송
        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)
        
        # Nav2 Goal 토픽 이름 (보통 'goal_pose')
        self.goal_topic = self.declare_parameter("goal_topic", "goal_pose").value

        # ------------------------------------------
        # [핵심] Namespace(접두사) 처리
        # 멀티 로봇 환경이므로, 토픽 앞에 "/로봇이름"을 붙여 구분합니다.
        # 예: "/wc1/odom", "/wc2/odom"
        # ------------------------------------------
        self.topic_prefix = f"/{self.robot_name}"
        
        # ------------------------------------------
        # 3. 내부 상태 변수 초기화
        # ------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery_percent = 90
        self.current_state = STATE_WAITING 

        # 네트워크 관련 변수
        self.sock = None
        self.lock = threading.Lock() # 스레드 간 충돌 방지용 뮤텍스
        self.logged_in = False       # 로그인(이름 전송) 완료 여부
        self.running = True          # 프로그램 실행 상태 플래그
        
        # 재접속(Backoff) 알고리즘 변수
        self.backoff = 1.0           # 현재 대기 시간 (초)
        self.backoff_max = 10.0      # 최대 대기 시간 (초)
        self.next_connect_time = 0.0 # 다음 접속 시도 가능 시각

        # ------------------------------------------
        # 4. ROS 2 토픽 구독(Sub) 및 발행(Pub) 설정
        # ------------------------------------------
        
        # (1) 위치 정보 구독 (AMCL 또는 Odom)
        if self.use_amcl_pose:
            self.create_subscription(PoseWithCovarianceStamped, f"{self.topic_prefix}/amcl_pose", self.pose_cb, 10)
        else:
            self.create_subscription(Odometry, f"{self.topic_prefix}/odom", self.odom_pose_cb, 10)

        # (2) 배터리 정보 구독
        self.create_subscription(BatteryState, f"{self.topic_prefix}/battery_state", self.batt_cb, 10)
        
        # (3) 목표 지점 발행 (서버 -> ROS -> Nav2)
        self.goal_pub = self.create_publisher(PoseStamped, f"{self.topic_prefix}/{self.goal_topic}", 10)

        # ------------------------------------------
        # 5. 타이머 및 수신 스레드 시작
        # ------------------------------------------
        
        # 주기적으로 서버에 데이터를 보내는 타이머 (TX)
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)

        # 서버로부터 데이터를 계속 읽어오는 별도 스레드 (RX)
        # 메인 스레드는 ROS spin을 돌려야 하므로, 수신은 별도 스레드에서 처리함
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 시작 로그 출력
        self.get_logger().info(f"Bridge Started for [{self.robot_name}] -> Server {self.server_ip}:{self.server_port}")

    # =========================================================
    # [Helper] 유틸리티 함수들
    # =========================================================

    def get_state_name(self, state_id):
        """숫자로 된 상태 ID를 문자열로 변환 (로그 출력용)"""
        names = {
            STATE_WAITING: "WAITING", STATE_HEADING: "HEADING", STATE_BOARDING: "BOARDING",
            STATE_RUNNING: "RUNNING", STATE_STOP: "STOP", STATE_ARRIVED: "ARRIVED",
            STATE_EXITING: "EXITING", STATE_CHARGING: "CHARGING", STATE_ERROR: "ERROR"
        }
        return names.get(state_id, "UNKNOWN")

    def change_state(self, new_state):
        """로봇 상태 변경 및 로그 출력"""
        if self.current_state != new_state:
            old_name = self.get_state_name(self.current_state)
            new_name = self.get_state_name(new_state)
            self.get_logger().info(f"State Change: {old_name} -> {new_name}")
            self.current_state = new_state

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """쿼터니언(x,y,z,w)을 2D 방향각(Yaw, 라디안)으로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """2D 방향각(Yaw)을 쿼터니언으로 변환"""
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.z = math.sin(yaw * 0.5)
        return q

    # =========================================================
    # [ROS Callbacks] 토픽 수신 처리 함수들
    # =========================================================
    
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL 위치 데이터 수신 시 호출"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_pose_cb(self, msg: Odometry):
        """오도메트리 데이터 수신 시 호출"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def batt_cb(self, msg: BatteryState):
        """배터리 상태 수신 시 호출"""
        if msg.percentage is not None and msg.percentage >= 0.0:
            # 0.0~1.0 범위라면 100을 곱하고, 0~100 범위라면 그대로 사용
            p = int(msg.percentage * 100.0) if msg.percentage <= 1.0 else int(msg.percentage)
            self.battery_percent = max(0, min(100, p))

    # =========================================================
    # [Network] TCP 소켓 관리 함수들
    # =========================================================

    def _set_keepalive(self, s: socket.socket):
        """소켓 연결이 끊어졌는지 감지하기 위한 Keepalive 설정"""
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except Exception:
            pass

    def connect(self) -> bool:
        """서버 접속 시도 (재접속 대기 로직 포함)"""
        now = time.time()
        # 아직 재접속 대기 시간이 안 지났으면 스킵
        if now < self.next_connect_time:
            return False

        with self.lock:
            if self.sock: return True # 이미 연결됨

            try:
                # 소켓 생성 및 접속
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0) # 3초간 응답 없으면 타임아웃
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None) # 접속 후에는 블로킹 모드로 전환
                
                self.sock = s
                self.logged_in = False # 접속은 했지만 아직 로그인은 안 함
                
                # 접속 성공 시 재접속 변수 초기화
                self.backoff = 1.0
                self.next_connect_time = 0.0
                self.get_logger().info("TCP Connected to Server")
                return True
            
            except Exception as e:
                # 접속 실패 시 소켓 정리 및 대기 시간(Backoff) 증가
                if self.sock: s.close()
                self.sock = None
                self.logged_in = False
                self.next_connect_time = now + self.backoff
                # 대기 시간 2배씩 증가 (최대 10초)
                self.backoff = min(self.backoff * 2.0, self.backoff_max)
                return False

    def close_socket(self, reason: str):
        """소켓 안전 종료"""
        with self.lock:
            if self.sock:
                try: self.sock.close()
                except Exception: pass
            self.sock = None
            self.logged_in = False
        self.get_logger().warn(f"Socket closed: {reason}")

    # =========================================================
    # [TX/RX Logic] 송수신 및 패킷 처리
    # =========================================================

    def build_packet(self, msg_type: int, payload: bytes) -> bytes:
        """
        [패킷 생성]
        헤더(4바이트) + 데이터(Payload)를 합쳐서 반환합니다.
        C 구조체: struct { magic, device, type, len }
        """
        if len(payload) > 255:
            self.get_logger().error("Payload too large!")
            return b""
            
        # struct.pack을 이용해 헤더 정보를 바이너리로 변환
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        return header + payload

    def send_packet(self, msg_type: int, payload: bytes):
        """실제 소켓으로 데이터를 전송하는 함수"""
        pkt = self.build_packet(msg_type, payload)
        if not pkt: return
        
        with self.lock:
            if not self.sock: return
            try:
                self.sock.sendall(pkt)
            except Exception as e:
                self.close_socket(f"Send Error: {e}")

    def recvall(self, sock: socket.socket, n: int) -> bytes:
        """
        [중요] TCP 데이터 수신 헬퍼
        TCP는 스트림 방식이므로, n바이트를 달라고 해도 한 번에 다 안 올 수 있습니다.
        n바이트가 꽉 찰 때까지 반복해서 읽어야 합니다.
        """
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk: return b"" # 연결 끊김
                data += chunk
            except Exception:
                return b""
        return data

    def rx_loop(self):
        """
        [수신 스레드]
        서버로부터 오는 데이터를 무한 루프로 대기합니다.
        """
        while self.running and rclpy.ok():
            with self.lock:
                current_sock = self.sock
            
            # 연결 안 되어 있으면 1초 대기 후 재확인
            if current_sock is None:
                time.sleep(1.0)
                continue

            try:
                # 1. 헤더 읽기 (고정 4바이트)
                header_data = self.recvall(current_sock, HDR_SIZE)
                if len(header_data) != HDR_SIZE:
                    self.close_socket("Remote close (Header)")
                    continue

                # 헤더 파싱 (unpack)
                magic, dev, msg_type, payload_len = struct.unpack(HDR_FMT, header_data)
                
                # 매직 넘버 확인 (잘못된 패킷 필터링)
                if magic != MAGIC_NUMBER:
                    self.get_logger().warn(f"Invalid Magic: {magic}")
                    continue

                # 2. 페이로드(데이터) 읽기
                payload_data = b""
                if payload_len > 0:
                    payload_data = self.recvall(current_sock, payload_len)
                    if len(payload_data) != payload_len:
                        self.close_socket("Remote close (Payload)")
                        continue

                # 3. 메시지 처리 핸들러 호출
                self.handle_server_message(msg_type, payload_data)

            except Exception as e:
                self.close_socket(f"RX Exception: {e}")
                time.sleep(1.0)

    def handle_server_message(self, msg_type, payload):
        """서버 메시지 타입별 처리 로직"""
        
        # [CASE] 서버가 목표 지점을 할당했을 때 (MSG_ASSIGN_GOAL)
        if msg_type == MSG_ASSIGN_GOAL:
            # 데이터 길이 검증
            if len(payload) != GOAL_SIZE:
                self.get_logger().warn(f"Goal size mismatch: expected {GOAL_SIZE}, got {len(payload)}")
                return

            # 바이너리를 float(x, y)로 변환
            gx, gy = struct.unpack(GOAL_FMT, payload)
            self.get_logger().info(f"★ Received Goal from Server: ({gx:.2f}, {gy:.2f})")

            # ROS Nav2 메시지 생성 (PoseStamped)
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map" # 지도 좌표계 기준
            goal.pose.position.x = float(gx)
            goal.pose.position.y = float(gy)
            goal.pose.orientation = self.yaw_to_quaternion(0.0) # 방향은 기본값(동쪽)

            # Nav2로 목표 발행
            self.goal_pub.publish(goal)
            self.get_logger().info(f"-> Published to {self.goal_pub.topic_name}")
            
            # 상태를 '이동 중(HEADING)'으로 변경
            if self.current_state == STATE_WAITING:
                self.change_state(STATE_HEADING)

    def send_login_once(self):
        """최초 접속 시 '저 누구에요'라고 이름표 보내기"""
        if self.logged_in: return
        
        # 이름 길이 제한 (안전하게 64바이트까지만)
        name_bytes = self.robot_name.encode("utf-8")[:64]
        
        # 로그인 요청 패킷 전송
        self.send_packet(MSG_LOGIN_REQ, name_bytes)
        self.logged_in = True
        self.get_logger().info(f"Sent LOGIN_REQ: {self.robot_name}")

    def tx_timer_cb(self):
        """
        [송신 타이머]
        설정된 주기에 따라 서버로 로봇의 현재 상태(배터리, 위치)를 보냅니다.
        """
        # 접속 안 되어 있으면 재접속 시도
        if not self.connect(): return

        try:
            # 1. 로그인 안했으면 로그인 패킷부터 전송
            self.send_login_once()

            # 2. 상태 패킷 생성 (C 구조체 RobotStateData와 일치)
            # 포맷: <ifffB (17 bytes)
            payload = struct.pack(
                STATE_FMT,
                int(self.battery_percent), # 배터리 (int)
                float(self.x),             # 현재 X (float)
                float(self.y),             # 현재 Y (float)
                float(self.theta),         # 현재 방향 (float)
                int(self.current_state)    # 상태 코드 (uint8)
            )
            
            # 전송
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            self.get_logger().error(f"TX Fail: {e}")
            self.close_socket("TX Error")


def main():
    """프로그램 시작점"""
    rclpy.init()

    # ----------------------------------------------------
    # [핵심] C 서버가 넘겨준 로봇 이름 인자 받기
    # C 코드에서: execlp("python3", "ros2_bridge.py", "tb1", NULL)
    # sys.argv[0]: 스크립트 이름
    # sys.argv[1]: 첫 번째 인자 (로봇 이름)
    # ----------------------------------------------------
    robot_name_arg = "wc1" # 인자가 없을 경우 기본값
    if len(sys.argv) > 1:
        robot_name_arg = sys.argv[1]

    print(f"--- Starting TCP Bridge for [{robot_name_arg}] ---")

    # 파라미터 강제 주입 (노드 생성 시 robot_name 파라미터 자동 설정)
    param_override = Parameter("robot_name", Parameter.Type.STRING, robot_name_arg)
    
    # 노드 생성
    node = TcpBridge(parameter_overrides=[param_override])
    
    try:
        # 노드 실행 (무한 루프)
        rclpy.spin(node)
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