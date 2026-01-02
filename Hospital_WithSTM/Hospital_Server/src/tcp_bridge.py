#!/usr/bin/env python3
"""
파일명: tcp_bridge.py (Server PC용)
수정내용: DB 직접 접속 제거 -> map_graph.json 파일 기반으로 통합
"""

import sys
import socket
import struct
import threading
import time
import math
import json
import heapq

# ROS 2 관련
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# 메시지 타입
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool, String

# =========================================================================
# 1. 프로토콜 및 상수
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

BTN_NONE = 0
BTN_BOARDING_COMPLETE = 2
BTN_RESUME = 3
BTN_EMERGENCY = 4
BTN_EXIT_COMPLETE = 5

HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)
STATE_FMT = "<ifffBiB" 
GOAL_FMT = "<iffff64s"
GOAL_SIZE = struct.calcsize(GOAL_FMT)

# =========================================================================
# [통합] 맵 데이터 매니저 (길찾기 + 장소명 변환)
# =========================================================================
class MapManager:
    def __init__(self, json_path):
        self.nodes = {}
        self.edges = {}
        self.locations = {} # {'장소명': (x,y), ...}
        self.load_map(json_path)

    def load_map(self, json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 1. 노드 로드
            self.nodes = {int(k): tuple(v) for k, v in data.get('nodes', {}).items()}
            
            # 2. 엣지 로드
            self.edges = {}
            for u, v, w in data.get('edges', []):
                self.edges.setdefault(u, []).append((v, w))
                self.edges.setdefault(v, []).append((u, w))
            
            # 3. [New] 장소 정보 로드
            # JSON: "locations": {"정형외과": [1.0, 2.0], ...}
            self.locations = {k: tuple(v) for k, v in data.get('locations', {}).items()}
            
            print(f"🗺️  Map Loaded: {len(self.nodes)} nodes, {len(self.locations)} locations")
            
        except Exception as e:
            print(f"⚠️  Map file error: {e}")

    def get_location_name(self, x, y):
        """ 좌표(x,y)와 가장 가까운 장소 이름 반환 """
        if not self.locations: return "알수없음"
        
        # 거리 계산해서 가장 가까운 장소 찾기
        closest_name = min(self.locations.keys(), 
                           key=lambda name: math.dist((x, y), self.locations[name]))
        
        # 거리가 너무 멀면(1m 이상) 유효하지 않은 것으로 간주
        dist = math.dist((x, y), self.locations[closest_name])
        if dist < 1.0:
            return closest_name
        else:
            return "복도/이동중"

    def get_path(self, sx, sy, gx, gy):
        """ A* 알고리즘 길찾기 """
        if not self.nodes: return [(gx, gy)]
        
        # 시작/끝 좌표와 가장 가까운 노드 찾기
        start_node = min(self.nodes.keys(), key=lambda k: math.dist((sx, sy), self.nodes[k]))
        end_node = min(self.nodes.keys(), key=lambda k: math.dist((gx, gy), self.nodes[k]))
        
        queue = [(0, start_node, [])]
        visited = set()
        
        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            if curr in visited: continue
            visited.add(curr)
            path = path + [curr]
            
            if curr == end_node:
                # 노드 좌표들 + 최종 목적지 좌표
                return [self.nodes[n] for n in path] + [(gx, gy)]
            
            for neighbor, w in self.edges.get(curr, []):
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + w + math.dist(self.nodes[neighbor], self.nodes[end_node]), neighbor, path))
        
        return [(gx, gy)] # 길 못 찾으면 직선 이동

# =========================================================================
# 3. 메인 노드
# =========================================================================
class TcpBridge(Node):
    def __init__(self):
        super().__init__("tcp_bridge")
        
        # 파라미터
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = 8080
        self.robot_name = "wc1"
        map_file = "map_graph.json"

        # 변수 초기화
        self.current_state = STATE_WAITING
        self.battery = 100
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.ultra_distance = 0
        self.seat_detected = False
        
        self.current_caller = ""
        self.start_loc_name = ""
        self.dest_loc_name = ""
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0
        
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.waypoint_queue = []
        
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        self.backoff = 1.0
        self.next_connect_time = 0.0

        # [Modified] MapManager 하나로 통합
        self.map_mgr = MapManager(map_file)

        # ROS 통신
        prefix = f"/{self.robot_name}"
        self.create_subscription(Odometry, f"{prefix}/odom", self.odom_pose_cb, 10)
        self.create_subscription(BatteryState, f"{prefix}/battery_state", self.batt_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/ultrasonic", self.ultra_cb, 10)
        self.create_subscription(Bool, f"{prefix}/stm32/seat", self.seat_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/button", self.button_cb, 10)

        self.ui_pub = self.create_publisher(String, f"{prefix}/ui/info", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.caller_pub = self.create_publisher(String, f"{prefix}/caller_name", 10)

        # 타이머 & 스레드
        self.create_timer(0.5, self.tx_timer_cb)
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"💻 PC TcpBridge Started ({self.robot_name})")

    # --- Callbacks ---
    def odom_pose_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Theta 변환 생략

    def batt_cb(self, msg): self.battery = int(msg.percentage)
    def ultra_cb(self, msg): self.ultra_distance = msg.data
    def seat_cb(self, msg): self.seat_detected = msg.data

    def button_cb(self, msg):
        btn = msg.data
        self.get_logger().info(f"🔘 Button: {btn}")
        self.handle_button_logic(btn)

    # --- Logic (FSM) ---
    def handle_button_logic(self, btn):
        if self.current_state == STATE_BOARDING:
            if btn == BTN_BOARDING_COMPLETE:
                if self.seat_detected:
                    self.get_logger().info("✅ 탑승 완료! 출발!")
                    self.change_state(STATE_RUNNING)
                    self.start_path_navigation(self.final_goal_x, self.final_goal_y)
                else:
                    self.get_logger().warn("⚠️ 환자 미탑승 (FSR Check Fail)")

        elif self.current_state == STATE_RUNNING:
            if btn == BTN_EMERGENCY:
                self.get_logger().warn("🚨 비상 정지!")
                self.prev_state = self.current_state
                self.change_state(STATE_STOP)
                self.stop_nav2()

        elif self.current_state == STATE_STOP:
            if btn == BTN_RESUME:
                self.get_logger().info("▶️ 동작 재개")
                self.change_state(self.prev_state)
                self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

        elif self.current_state == STATE_ARRIVED:
            if btn == BTN_EXIT_COMPLETE:
                if not self.seat_detected:
                    self.get_logger().info("✅ 하차 완료! 대기 모드.")
                    self.change_state(STATE_WAITING)
                    self.current_caller = ""; self.start_loc_name = ""; self.dest_loc_name = ""
                    self.publish_ui_info()
                else:
                    self.get_logger().warn("⚠️ 환자 착석 중 (FSR Check Fail)")

    def change_state(self, new_state):
        self.current_state = new_state
        self.publish_ui_info()

    # --- UI & Nav Helper ---
    def publish_ui_info(self):
        s_mode = str(self.current_state)
        s_caller = self.current_caller if self.current_caller else "대기중"
        s_start = self.start_loc_name if self.start_loc_name else "-"
        s_dest = self.dest_loc_name if self.dest_loc_name else "-"
        msg = f"{s_mode}@{s_caller}@{s_start}@{s_dest}"
        self.ui_pub.publish(String(data=msg))

    def publish_nav2_goal(self, x, y):
        if self.current_state == STATE_STOP: return
        self.current_goal_x = x; self.current_goal_y = y
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x; goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def stop_nav2(self):
        self.waypoint_queue = []
        self.publish_nav2_goal(self.x, self.y)

    def start_path_navigation(self, tx, ty):
        path = self.map_mgr.get_path(self.x, self.y, tx, ty)
        self.waypoint_queue = path
        self.pop_and_drive()

    def pop_and_drive(self):
        if self.waypoint_queue:
            wp = self.waypoint_queue.pop(0)
            self.publish_nav2_goal(wp[0], wp[1])

    # --- Network ---
    def handle_server_message(self, msg_type, payload):
        if msg_type == MSG_ASSIGN_GOAL:
            if len(payload) != GOAL_SIZE: return
            order, sx, sy, gx, gy, raw_name = struct.unpack(GOAL_FMT, payload)
            
            try: caller = raw_name.split(b'\x00')[0].decode('utf-8')
            except: caller = "Unknown"
            
            if order == 99: self.destroy_node(); sys.exit(0)

            self.get_logger().info(f"📩 Order {order}: {caller}")
            self.current_caller = caller
            self.caller_pub.publish(String(data=caller))

            # [Modified] JSON에서 이름 찾기
            self.start_loc_name = self.map_mgr.get_location_name(sx, sy)
            self.dest_loc_name = self.map_mgr.get_location_name(gx, gy)
            self.final_goal_x = gx; self.final_goal_y = gy

            if order == 6: # 배차
                self.change_state(STATE_HEADING)
                self.start_path_navigation(sx, sy)
            elif order in [1, 4, 5]: 
                self.change_state(STATE_RUNNING)
                self.start_path_navigation(gx, gy)

    def tx_timer_cb(self):
        if not self.connect(): return
        self.send_login_once()
        self.publish_ui_info() # 주기적 갱신

        # 도착 판정
        dist = math.dist((self.x, self.y), (self.current_goal_x, self.current_goal_y))
        if self.current_state in [STATE_HEADING, STATE_RUNNING] and dist < 0.3:
            if self.waypoint_queue:
                self.pop_and_drive()
            else:
                if self.current_state == STATE_HEADING:
                    self.get_logger().info("🏁 출발지 도착")
                    self.change_state(STATE_BOARDING)
                elif self.current_state == STATE_RUNNING:
                    self.get_logger().info("🏁 목적지 도착")
                    self.change_state(STATE_ARRIVED)

        # 서버 보고
        try:
            payload = struct.pack(STATE_FMT,
                int(self.battery), self.x, self.y, self.theta,
                int(self.current_state),
                int(self.ultra_distance), int(1 if self.seat_detected else 0)
            )
            self.send_packet(MSG_ROBOT_STATE, payload)
        except: self.close_socket("TX Error")

    # ... 소켓 기본 함수들은 이전과 동일 (connect, send_packet, recvall 등) ...
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
    robot_name = "wc1" if len(sys.argv) < 2 else sys.argv[1]
    node = TcpBridge()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()