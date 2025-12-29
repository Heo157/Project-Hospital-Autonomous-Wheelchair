"""
============================================================================
 파일명: tcp_bridge.py
 설명:   ROS 2(Nav2) <-> TCP(C Server) 간의 통신 중계 및 FSM 시나리오 제어기
 작성일: 2025-12-28 (Fix: AMCL QoS Transient Local)
 
 [해결된 문제]
 - ros2 topic echo에서는 데이터가 보이나 코드에서는 안 보이는 문제 해결
 - 원인: AMCL의 Transient Local QoS와 코드의 Volatile QoS 불일치 -> 수정 완료
 
 [기능]
 1. Publisher: "/goal_pose"
 2. Subscriber: "/amcl_pose" (Transient Local 적용)
 3. Debug: [RX] 로그 출력으로 데이터 수신 확인
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

# [필수] QoS 설정을 위한 모듈
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# 메시지 타입
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# ==========================================
# 1. 프로토콜 & 상수 정의
# ==========================================

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
STATE_FMT = "<ifffB"
STATE_SIZE = struct.calcsize(STATE_FMT)
GOAL_FMT = "<iffff"
GOAL_SIZE = struct.calcsize(GOAL_FMT)


class TcpBridge(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("tcp_bridge", parameter_overrides=parameter_overrides)

        # ------------------------------------------
        # 1. 파라미터 & 변수 초기화
        # ------------------------------------------
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = int(self.declare_parameter("server_port", 8080).value)
        self.robot_name = self.declare_parameter("robot_name", "wc1").value
        self.use_amcl_pose = bool(self.declare_parameter("use_amcl_pose", True).value)
        self.tx_hz = float(self.declare_parameter("tx_hz", 2.0).value)

        # (참고) 지금 터틀봇은 namespace가 "/"라서 prefix 필요 없음
        self.topic_prefix = f"/{self.robot_name}"

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery_percent = 90

        self.current_state = STATE_WAITING
        self.prev_state = STATE_WAITING
        self.mission_mode = "NONE"

        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0

        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        self.backoff = 1.0
        self.next_connect_time = 0.0

        # ------------------------------------------
        # 2. ROS 2 통신 설정 (QoS: Transient Local)
        # ------------------------------------------

        # [QoS 설정] 터틀봇 /amcl_pose QoS에 맞춤
        #   Reliability: RELIABLE
        #   Durability:  TRANSIENT_LOCAL
        #   Depth: 1
        amcl_qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # (1) 위치 정보 구독
        if self.use_amcl_pose:
            # ✅ FIX: /server_pose 구독 제거, /amcl_pose + transient local QoS 적용
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self.pose_cb,
                amcl_qos_profile
            )
        else:
            self.create_subscription(Odometry, "/odom", self.odom_pose_cb, 10)

        # (2) 배터리 정보 구독
        self.create_subscription(BatteryState, "/battery_state", self.batt_cb, 10)

        # (3) 목표 지점 발행
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # ------------------------------------------
        # 3. 타이머 & 스레드 시작
        # ------------------------------------------
        period = 1.0 / max(0.1, self.tx_hz)
        self.create_timer(period, self.tx_timer_cb)

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"FSM Bridge Started for [{self.robot_name}] - Waiting for AMCL Data...")

    # =========================================================
    # [ROS Callbacks]
    # =========================================================
    def pose_cb(self, msg):
        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y

        # [데이터 수신 확인용 로그]
        self.get_logger().info(f"[RX] AMCL: x={cur_x:.2f}, y={cur_y:.2f}")

        self.x = float(cur_x)
        self.y = float(cur_y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom_pose_cb(self, msg):
        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y
        self.get_logger().info(f"[RX] ODOM: x={cur_x:.2f}, y={cur_y:.2f}")

        self.x = float(cur_x)
        self.y = float(cur_y)
        self.theta = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def batt_cb(self, msg):
        if msg.percentage is not None and msg.percentage >= 0.0:
            p = int(msg.percentage * 100.0) if msg.percentage <= 1.0 else int(msg.percentage)
            self.battery_percent = max(0, min(100, p))

    # =========================================================
    # [Helper] 유틸리티 함수
    # =========================================================
    def get_state_name(self, state_id):
        names = {
            STATE_WAITING: "WAITING", STATE_HEADING: "HEADING", STATE_BOARDING: "BOARDING",
            STATE_RUNNING: "RUNNING", STATE_STOP: "STOP", STATE_ARRIVED: "ARRIVED",
            STATE_EXITING: "EXITING", STATE_CHARGING: "CHARGING", STATE_ERROR: "ERROR"
        }
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

    # =========================================================
    # [Nav2 Control] 주행 명령
    # =========================================================
    def publish_nav2_goal(self, x, y):
        if self.current_state == STATE_STOP:
            self.get_logger().warn("Cannot move in STOP state!")
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
        self.get_logger().info(f"Nav2 Goal -> ({x:.2f}, {y:.2f})")

    def stop_nav2(self):
        self.publish_nav2_goal(self.x, self.y)

    # =========================================================
    # [Scenario] 시나리오 단계별 콜백
    # =========================================================
    def step_boarding_complete(self):
        self.get_logger().info("✅ Passenger Boarded. Heading to Destination.")
        self.change_state(STATE_RUNNING)
        self.mission_mode = "DELIVER"
        self.publish_nav2_goal(self.final_goal_x, self.final_goal_y)

    def step_exiting_complete(self):
        self.get_logger().info("✅ Passenger Exited. Mission Complete.")
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"

    # =========================================================
    # [Logic] 서버 메시지 처리
    # =========================================================
    def handle_server_message(self, msg_type, payload):
        if msg_type == MSG_ASSIGN_GOAL:
            if len(payload) != GOAL_SIZE:
                return

            order, sx, sy, gx, gy = struct.unpack(GOAL_FMT, payload)

            self.get_logger().info(f"CMD Order={order} | Start({sx:.1f},{sy:.1f}) Goal({gx:.1f},{gy:.1f})")

            if order in [1, 4]:
                self.publish_nav2_goal(gx, gy)
                self.change_state(STATE_RUNNING)
                self.mission_mode = "NONE"

            elif order == 5:
                self.publish_nav2_goal(gx, gy)
                self.change_state(STATE_RUNNING)
                self.mission_mode = "CHARGE"

            elif order == 6:
                self.publish_nav2_goal(sx, sy)
                self.final_goal_x = gx
                self.final_goal_y = gy
                self.change_state(STATE_HEADING)
                self.mission_mode = "PICKUP"

            elif order == 2:
                if self.current_state != STATE_STOP:
                    self.prev_state = self.current_state
                    self.change_state(STATE_STOP)
                    self.stop_nav2()

            elif order == 3:
                if self.current_state == STATE_STOP:
                    self.get_logger().info("Resuming operation...")
                    self.change_state(self.prev_state)
                    self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

    # =========================================================
    # [Timer] 주기적 상태 체크 및 전송
    # =========================================================
    def tx_timer_cb(self):
        if not self.connect():
            return
        self.send_login_once()

        try:
            dist = math.sqrt((self.x - self.current_goal_x)**2 + (self.y - self.current_goal_y)**2)

            if self.current_state == STATE_HEADING and dist < 0.2:
                self.get_logger().info("🚩 Arrived at Start. Boarding (Wait 5s)...")
                self.change_state(STATE_BOARDING)
                threading.Timer(5.0, self.step_boarding_complete).start()

            elif self.current_state == STATE_RUNNING and dist < 0.2:
                self.get_logger().info("🚩 Arrived at Destination.")
                self.change_state(STATE_ARRIVED)

                if self.mission_mode == "DELIVER":
                    self.get_logger().info("State: ARRIVED -> EXITING (Wait 5s)...")
                    self.change_state(STATE_EXITING)
                    threading.Timer(5.0, self.step_exiting_complete).start()
                    self.mission_mode = "DONE_WAIT"

                elif self.mission_mode == "CHARGE":
                    self.get_logger().info("State: ARRIVED -> CHARGING")
                    self.change_state(STATE_CHARGING)
                    self.mission_mode = "DONE_CHARGE"

                elif self.mission_mode == "NONE":
                    self.get_logger().info("State: ARRIVED -> WAITING")
                    self.change_state(STATE_WAITING)

            payload = struct.pack(
                STATE_FMT,
                int(self.battery_percent),
                float(self.x), float(self.y), float(self.theta),
                int(self.current_state)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)

        except Exception as e:
            self.get_logger().error(f"TX Fail: {e}")
            self.close_socket("TX Error")

    # =========================================================
    # [Network] 소켓 통신
    # =========================================================
    def _set_keepalive(self, s):
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except:
            pass

    def connect(self):
        now = time.time()
        if now < self.next_connect_time:
            return False

        with self.lock:
            if self.sock:
                return True
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._set_keepalive(s)
                s.settimeout(3.0)
                s.connect((self.server_ip, self.server_port))
                s.settimeout(None)
                self.sock = s
                self.logged_in = False
                self.backoff = 1.0
                self.next_connect_time = 0.0
                return True
            except:
                try:
                    s.close()
                except:
                    pass
                self.sock = None
                self.logged_in = False
                self.next_connect_time = now + self.backoff
                self.backoff = min(self.backoff * 2.0, 60.0)
                return False

    def close_socket(self, reason):
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
            self.sock = None
            self.logged_in = False
        self.get_logger().warn(f"Socket closed: {reason}")

    def send_packet(self, msg_type, payload):
        if len(payload) > 255:
            return
        header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, msg_type, len(payload))
        with self.lock:
            if not self.sock:
                return
            try:
                self.sock.sendall(header + payload)
            except Exception as e:
                self.close_socket(f"Send Error: {e}")

    def send_login_once(self):
        if self.logged_in:
            return
        self.send_packet(MSG_LOGIN_REQ, self.robot_name.encode("utf-8")[:64])
        self.logged_in = True
        self.get_logger().info(f"Sent LOGIN_REQ: {self.robot_name}")

    def recvall(self, sock, n):
        data = b""
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk:
                    return b""
                data += chunk
            except:
                return b""
        return data

    def rx_loop(self):
        while self.running and rclpy.ok():
            with self.lock:
                sock = self.sock
            if sock is None:
                time.sleep(1.0)
                continue
            try:
                hdr = self.recvall(sock, HDR_SIZE)
                if len(hdr) != HDR_SIZE:
                    self.close_socket("Header Error")
                    continue
                magic, dev, msg_type, length = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER:
                    continue

                payload = b""
                if length > 0:
                    payload = self.recvall(sock, length)
                    if len(payload) != length:
                        self.close_socket("Payload Error")
                        continue
                self.handle_server_message(msg_type, payload)

            except Exception as e:
                self.close_socket(f"RX Error: {e}")
                time.sleep(1.0)


def main():
    rclpy.init()
    robot_name = "wc1"
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

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
