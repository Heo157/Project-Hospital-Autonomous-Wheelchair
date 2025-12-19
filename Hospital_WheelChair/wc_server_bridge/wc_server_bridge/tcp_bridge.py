import socket
import struct
import threading
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# ==========================================
# 1. 프로토콜 정의 (common_defs.h 헤더와 일치)
# ==========================================
# 패킷의 시작을 알리는 매직 넘버 (수신 측에서 유효성 검증용)
MAGIC_NUMBER = 0xAB

# 장치 타입 식별자 (0x02: ROS 로봇)
DEVICE_ROBOT_ROS = 0x02

# 메시지 타입 정의
MSG_LOGIN_REQ   = 0x01  # 로그인 요청 (로봇 이름 전송)
MSG_ROBOT_STATE = 0x20  # 로봇 상태 보고 (위치, 배터리 등)

# ------------------------------------------
# 패킷 헤더 (PacketHeader) 정의: 4 Bytes
# 포맷: < (리틀 엔디안)
# B (Magic), B (DeviceType), B (MsgType), B (PayloadLength)
# ------------------------------------------
HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)  # 크기: 4바이트

# ------------------------------------------
# 로봇 상태 데이터 (RobotStateData) 정의: 17 Bytes
# 포맷: < (리틀 엔디안)
# i (배터리, int 4B), f (x좌표, float 4B), f (y좌표, float 4B), 
# f (theta, float 4B), B (이동여부, uchar 1B)
# ------------------------------------------
STATE_FMT = "<ifffB"
STATE_SIZE = struct.calcsize(STATE_FMT)  # 크기: 17바이트


class TcpBridge(Node):
    def __init__(self):
        super().__init__("tcp_bridge")

        # ------------------------------------------
        # 2. 파라미터 설정 (ROS 2 파라미터 서버 사용)
        # ------------------------------------------
        # 실행 시 --ros-args -p key:=value 형태로 변경 가능
        self.server_ip = self.declare_parameter("server_ip", "10.10.14.138").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)

        self.robot_name = self.declare_parameter("robot_name", "wc1").value  # 로봇 고유 식별자
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value) # 위치 소스 선택

        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)  # 전송 주기 (기본 2Hz = 0.5초)

        # ------------------------------------------
        # 3. 상태 변수 초기화
        # ------------------------------------------
        # 서버로 보낼 로봇의 현재 상태값들
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0           # 로봇이 바라보는 방향 (라디안)
        self.battery_percent = 100
        self.is_moving = 0

        # TCP 네트워크 관련 변수
        self.sock = None
        self.lock = threading.Lock() # 소켓 동시 접근 방지용 락
        self.logged_in = False       # 서버에 로그인(이름 전송) 완료 여부

        # 재접속(Backoff) 알고리즘 변수
        # 연결 실패 시 대기 시간을 점진적으로 늘려 서버 부하를 줄임
        self.backoff = 1.0
        self.backoff_max = 10.0
        self.next_connect_time = 0.0

        # ------------------------------------------
        # 4. 토픽 구독 (Subscription) 설정
        # ------------------------------------------
        # 위치 정보: AMCL(지도 기반) 또는 Odometry(오도메트리) 중 선택
        if self.use_amcl_pose:
            self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_cb, 10)
        else:
            self.create_subscription(Odometry, "/odom", self.odom_pose_cb, 10)

        # 속도 정보: 로봇이 움직이는지 판단하기 위함 (/odom 사용)
        self.create_subscription(Odometry, "/odom", self.odom_twist_cb, 10)
        
        # 배터리 정보
        self.create_subscription(BatteryState, "/battery_state", self.batt_cb, 10)

        # ------------------------------------------
        # 5. 전송 타이머 설정
        # ------------------------------------------
        # 설정된 Hz 주기에 맞춰 tx_timer_cb 실행
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)

        self.get_logger().info(
            f"Start Bridge: server={self.server_ip}:{self.server_port}, "
            f"robot={self.robot_name}, tx={self.tx_hz}Hz"
        )

    # ==========================================
    # Quaternion → Yaw 변환 함수
    # ==========================================
    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        Quaternion(x, y, z, w)을 Yaw 각도(라디안)로 변환
        - Yaw: Z축 회전 (2D 평면에서의 방향)
        - 범위: -π ~ π (-3.14 ~ 3.14)
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ==========================================
    # ROS 콜백 함수 (데이터 수집)
    # ==========================================
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL 위치 데이터 수신 시 호출"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        # Quaternion을 Yaw로 변환하여 저장
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_pose_cb(self, msg: Odometry):
        """Odom 위치 데이터 수신 시 호출 (AMCL 미사용 시)"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        # Quaternion을 Yaw로 변환하여 저장
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_twist_cb(self, msg: Odometry):
        """로봇의 속도(Twist)를 확인하여 움직임 여부(is_moving) 갱신"""
        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)
        # 선속도나 각속도가 임계값 이상이면 움직이는 것으로 간주 (1: 이동 중, 0: 정지)
        self.is_moving = 1 if (abs(vx) > 0.03 or abs(wz) > 0.15) else 0

    def batt_cb(self, msg: BatteryState):
        """배터리 상태 수신"""
        # percentage: 0.0~1.0 (일부 드라이버는 오류 시 -1 반환 가능)
        if msg.percentage is not None and msg.percentage >= 0.0:
            p = int(msg.percentage * 100.0)
            self.battery_percent = max(0, min(100, p))
        # 만약 -1이 들어오면 이전 값을 유지하는 것이 안전합니다.

    # ==========================================
    # TCP 네트워크 헬퍼 함수
    # ==========================================
    def _set_keepalive(self, s: socket.socket):
        """소켓의 Keepalive 옵션 활성화 (연결 끊김 감지용)"""
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except Exception:
            pass

    def connect(self) -> bool:
        """TCP 서버 연결 관리 (재접속 로직 포함)"""
        now = time.time()
        
        # 재접속 대기 시간(Backoff)이 지나지 않았으면 시도하지 않음
        if now < self.next_connect_time:
            return False

        with self.lock:
            # 이미 연결된 상태라면 True 반환
            if self.sock:
                return True

            try:
                # 소켓 생성 및 설정
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0)  # 연결 타임아웃 3초
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None) # 연결 후에는 블로킹 모드 전환

                # 연결 성공 시 초기화
                self.sock = s
                self.logged_in = False  # 새 연결이므로 다시 로그인 필요
                self.backoff = 1.0      # 백오프 시간 초기화
                self.next_connect_time = 0.0

                self.get_logger().info("TCP connected")
                return True

            except Exception as e:
                self.get_logger().warn(f"Connect failed: {e}")
                # 연결 실패 시 소켓 정리
                try:
                    s.close()
                except Exception:
                    pass
                self.sock = None
                self.logged_in = False

                # 다음 재접속 시도 시간 설정 (지수 백오프: 1초 -> 2초 -> 4초 ... 최대 10초)
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, self.backoff_max)
                return False

    def close_socket(self, reason: str):
        """소켓 연결을 안전하게 종료"""
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            self.logged_in = False
        self.get_logger().warn(f"Socket closed: {reason}")

    def build_packet(self, msg_type: int, payload: bytes) -> bytes:
        """헤더와 페이로드를 합쳐 전송할 바이너리 패킷 생성"""
        # payload 길이는 1바이트(uint8)로 표현되므로 최대 255바이트 제한
        if len(payload) > 255:
            raise ValueError("payload_len is uint8; max 255")
        
        # 헤더 패킹: Magic(1B) + Device(1B) + MsgType(1B) + Length(1B)
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        return header + payload

    def send_packet(self, msg_type: int, payload: bytes):
        """패킷 생성 및 전송"""
        pkt = self.build_packet(msg_type, payload)
        with self.lock:
            if not self.sock:
                raise ConnectionError("no socket")
            self.sock.sendall(pkt)

    # ==========================================
    # 메인 로직: 로그인 및 상태 전송
    # ==========================================
    def send_login_once(self):
        """서버 연결 직후, 로봇 이름을 포함한 로그인 패킷 전송 (1회성)"""
        if self.logged_in:
            return

        # 로봇 이름을 UTF-8 바이트로 변환
        name_bytes = self.robot_name.encode("utf-8")
        if len(name_bytes) == 0:
            raise ValueError("robot_name is empty")
        if len(name_bytes) > 32:
            # 이름이 너무 길면 자름 (운영 정책)
            name_bytes = name_bytes[:32]

        # 로그인 요청 패킷 전송
        self.send_packet(MSG_LOGIN_REQ, name_bytes)
        self.logged_in = True
        self.get_logger().info(f"LOGIN sent: {self.robot_name}")

    def tx_timer_cb(self):
        """
        [타이머 콜백] 주기적으로 실행되어 연결 관리 및 데이터 전송 수행
        1. 연결 시도 (connect)
        2. 로그인 패킷 전송 (최초 1회)
        3. 로봇 상태 패킷 전송 (반복)
        """
        # 1. 연결 상태 확인 및 재접속 시도
        if not self.connect():
            return

        try:
            # 2. 로그인 안 된 상태면 로그인 패킷 먼저 전송
            self.send_login_once()

            # 3. 로봇 상태 데이터 패킹 (theta 추가)
            payload = struct.pack(
                STATE_FMT,
                int(self.battery_percent),  # int32 (4B)
                float(self.x),              # float32 (4B)
                float(self.y),              # float32 (4B)
                float(self.theta),          # float32 (4B) - 바라보는 방향
                int(self.is_moving)         # uint8 (1B)
            )
            
            # 사이즈 검증 (실수 방지용)
            if len(payload) != STATE_SIZE:
                self.get_logger().error(f"Payload size mismatch: {len(payload)} != {STATE_SIZE}")
                return

            # 4. 상태 패킷 전송
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            # 전송 중 에러 발생 시 소켓 닫고 재연결 유도
            self.close_socket(f"TX failed: {e}")


def main():
    rclpy.init()
    node = TcpBridge()
    try:
        # 노드 실행 (콜백 루프)
        rclpy.spin(node)
    finally:
        # 종료 시 소켓 정리
        node.close_socket("shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()