#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================================
 파일명: tcp_bridge.py (Final Corrected Version)
 설명:   ROS 2(Nav2) <-> TCP(C Server) 통신 브리지
 
 [최종 수정 사항]
 1. 도착 판정 거리(DIST_TOLERANCE_FINAL)를 0.5m -> 0.8m로 대폭 완화
 2. control_loop 내부에 남은 거리 실시간 출력(Debug Print) 추가
 3. STM32 main.c 파싱 로직에 맞춰 UI 패킷 포맷 완전 수정
    - (기존) 상태@호출자@목적지@-
    - (수정) 상태@속도@배터리@호출자@출발지@목적지
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
# ROS 2 관련 라이브러리 임포트
# -------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool, String, Float32

# =========================================================================
# 1. 프로토콜 상수 및 설정 정의
# =========================================================================

# 패킷 유효성 검사용 매직 넘버
MAGIC_NUMBER = 0xAB
DEVICE_ROBOT_ROS = 0x02

# 메시지 타입 (서버와 약속된 프로토콜 ID)
MSG_LOGIN_REQ   = 0x01  # 로그인
MSG_ROBOT_STATE = 0x20  # 로봇 상태 보고
MSG_ASSIGN_GOAL = 0x30  # 작업 지시

# 로봇 상태 코드 (FSM)
STATE_WAITING  = 0
STATE_HEADING  = 1
STATE_BOARDING = 2
STATE_RUNNING  = 3
STATE_STOP     = 4
STATE_ARRIVED  = 5
STATE_EXITING  = 6
STATE_CHARGING = 7
STATE_ERROR    = 99

# 네트워크 패킷 포맷
HDR_FMT = "<BBBB"
HDR_SIZE = struct.calcsize(HDR_FMT)
STATE_FMT = "<ifffBiB"
GOAL_FMT = "<iffff64s"
GOAL_SIZE = struct.calcsize(GOAL_FMT)

# 하드웨어 버튼 매핑값
BTN_BOARDING_COMPLETE = 1
BTN_RESUME            = 3
BTN_EMERGENCY         = 4
BTN_EXIT_COMPLETE     = 5

# [설정] 주행 허용 오차 (단위: 미터)
# 중요: 도착 인식을 잘 못한다면 이 값을 늘려야 합니다. (0.5 -> 0.8)
DIST_TOLERANCE_FINAL    = 0.8  
# 경유지는 부드럽게 지나가도록 넓게 설정
DIST_TOLERANCE_WAYPOINT = 1.0  

# =========================================================================
# 2. 길찾기 및 맵 데이터 관리 클래스
# =========================================================================
class SimplePathFinder:
    def __init__(self, json_path):
        self.nodes = {}      
        self.edges = {}      
        self.locations = {}  
        self.load_map(json_path)

    def load_map(self, json_path):
        print(f"[Map] 맵 파일 로딩 시작: {json_path}")
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.nodes = {int(k): tuple(v) for k, v in data.get('nodes', {}).items()}
            
            for item in data.get('edges', []):
                if len(item) >= 3:
                    u, v, w = item[0], item[1], item[2]
                    self.edges.setdefault(u, []).append((v, w))
                    self.edges.setdefault(v, []).append((u, w))

            # 장소 이름 정보 로드
            raw_locs = data.get('locations', {})
            for name, coords in raw_locs.items():
                self.locations[name] = tuple(coords)

            print(f"[Map] 로딩 완료: 노드 {len(self.nodes)}개, 장소 {len(self.locations)}개")
            
        except Exception as e:
            print(f"[Map] ⚠️ 맵 로딩 실패: {e}")
            self.nodes = {}
            self.edges = {}
            self.locations = {}

    def find_location_name(self, target_x, target_y):
        if not self.locations: return "?"
        min_dist = 1.5
        found_name = "?"
        for name, coords in self.locations.items():
            dist = math.dist((target_x, target_y), coords)
            if dist < min_dist:
                min_dist = dist
                found_name = name
        return found_name

    def find_nearest_node(self, tx, ty):
        if not self.nodes: return None
        return min(self.nodes.keys(), key=lambda k: math.dist((tx, ty), self.nodes[k]))

    def get_path(self, sx, sy, gx, gy):
        if not self.nodes: return [(gx, gy)]
        start_node = self.find_nearest_node(sx, sy)
        end_node = self.find_nearest_node(gx, gy)
        
        if start_node is None or end_node is None: return [(gx, gy)]
        
        queue = [(0, start_node, [])]
        visited = set()
        
        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            if curr in visited: continue
            visited.add(curr)
            
            new_path = path + [curr]
            if curr == end_node:
                return [self.nodes[n] for n in new_path] + [(gx, gy)]
            
            for neighbor, weight in self.edges.get(curr, []):
                if neighbor not in visited:
                    h = math.dist(self.nodes[neighbor], self.nodes[end_node])
                    heapq.heappush(queue, (cost + weight + h, neighbor, new_path))
        
        return [(gx, gy)]

# =========================================================================
# 3. 메인 ROS 2 노드 클래스
# =========================================================================
class TcpBridge(Node):
    def __init__(self, robot_name_arg, map_file_arg):
        super().__init__("tcp_bridge")
        
        self.robot_name = robot_name_arg
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = 8080
        
        # 로봇 상태 정보
        self.current_state = STATE_WAITING
        self.mission_mode = "NONE"
        
        # 센서 데이터
        self.battery_percent = 100
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.seat_detected = False
        self.ultra_distance = 0
        
        # 네비게이션 목표
        self.final_goal_x = 0.0
        self.final_goal_y = 0.0
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.waypoint_queue = []
        
        # UI 표시 정보
        self.current_caller = ""     
        self.current_dest_name = "?" 
        
        # TCP 소켓
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        
        self.pathfinder = SimplePathFinder(map_file_arg)

        # ROS 설정
        prefix = f"/{self.robot_name}"
        self.create_subscription(Odometry, f"{prefix}/odom", self.odom_cb, 10)
        self.create_subscription(BatteryState, f"{prefix}/battery_state", self.batt_cb, 10)
        self.create_subscription(Float32, f"{prefix}/ultra_distance_cm", self.ultra_cb, 10)
        self.create_subscription(Bool, f"{prefix}/seat_detected", self.seat_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/button", self.button_cb, 10)
        
        self.ui_pub = self.create_publisher(String, f"{prefix}/ui/info", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.caller_pub = self.create_publisher(String, f"{prefix}/caller_name", 10)
        
        self.create_timer(0.5, self.control_loop)
        
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        
        print(f"\n🚀 [System] {self.robot_name} 시스템 시작됨.")
        print(f"   - 도착 판정 오차: {DIST_TOLERANCE_FINAL}m")

    # ---------------------------------------------------------------------
    # 콜백 함수
    # ---------------------------------------------------------------------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def batt_cb(self, msg):
        val = int(msg.percentage) if msg.percentage > 1.0 else int(msg.percentage * 100)
        self.battery_percent = val
        
    def ultra_cb(self, msg): self.ultra_distance = int(msg.data)
    def seat_cb(self, msg): self.seat_detected = msg.data

    def button_cb(self, msg):
        btn = msg.data
        if btn == 0: return
        print(f"\n[Button] 🔘 버튼 입력 감지: {btn}")
        
        if self.current_state == STATE_BOARDING and btn == BTN_BOARDING_COMPLETE:
            print("✅ 탑승 완료 확인. 목적지로 출발합니다.")
            self.change_state(STATE_RUNNING)
            self.start_path_navigation(self.final_goal_x, self.final_goal_y)
            
        elif self.current_state == STATE_ARRIVED and btn == BTN_EXIT_COMPLETE:
            print("✅ 임무 종료. 대기 모드로 복귀합니다.")
            self.reset_to_waiting()

        elif btn == BTN_EMERGENCY:
            print("🚨 비상 정지 명령!")
            self.change_state(STATE_STOP)
            self.publish_nav2_goal(self.x, self.y)

        elif self.current_state == STATE_STOP and btn == BTN_RESUME:
            print("▶️ 주행 재개.")
            self.change_state(STATE_RUNNING)
            self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

    # ---------------------------------------------------------------------
    # 로직 제어
    # ---------------------------------------------------------------------
    def change_state(self, new_state):
        if self.current_state != new_state:
            print(f"[State] 🔄 상태 전환: {self.current_state} -> {new_state}")
            self.current_state = new_state
        self.publish_ui_info()

    def control_loop(self):
        if not self.connect(): return
        self.send_login_once()
        self.publish_ui_info()
        self.send_robot_state()

        if self.current_state in [STATE_HEADING, STATE_RUNNING]:
            dist = math.dist((self.x, self.y), (self.current_goal_x, self.current_goal_y))
            
            # [Debug] 목표지점 근처(2m)에 오면 거리 정보를 계속 출력
            if dist < 2.0:
                print(f"   >>> 남은 거리: {dist:.3f}m (목표: {self.current_dest_name})")

            is_intermediate = (len(self.waypoint_queue) > 0)
            tolerance = DIST_TOLERANCE_WAYPOINT if is_intermediate else DIST_TOLERANCE_FINAL
            
            if dist < tolerance:
                if is_intermediate:
                    self.pop_and_drive() 
                else:
                    print(f"[Nav] 🏁 최종 목적지 도착 완료! (오차: {dist:.2f}m)")
                    self.handle_arrival()

    def handle_arrival(self):
        # 1. 로봇 정지
        self.publish_nav2_goal(self.x, self.y)

        # 2. 상태 전환 (강제 초기화 없이 대기)
        if self.current_state == STATE_HEADING:
            print("[Event] 픽업 위치 도착. 환자 탑승을 대기합니다.")
            self.change_state(STATE_BOARDING)
            
        elif self.current_state == STATE_RUNNING:
            print("[Event] 목적지 도착. 하차 또는 완료 버튼을 대기합니다.")
            self.change_state(STATE_ARRIVED)

    def reset_to_waiting(self):
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"
        self.current_caller = ""
        self.current_dest_name = "?"
        self.waypoint_queue = []
        print("[Logic] 시스템 대기 상태로 초기화됨.")

    def start_path_navigation(self, tx, ty):
        path = self.pathfinder.get_path(self.x, self.y, tx, ty)
        self.waypoint_queue = path
        self.current_dest_name = self.pathfinder.find_location_name(tx, ty)
        print(f"[Nav] 새로운 경로 시작: {self.current_dest_name} ({tx:.1f}, {ty:.1f})")
        self.pop_and_drive()

    def pop_and_drive(self):
        if self.waypoint_queue:
            wp = self.waypoint_queue.pop(0)
            self.publish_nav2_goal(wp[0], wp[1])

    def publish_nav2_goal(self, x, y):
        self.current_goal_x = x
        self.current_goal_y = y
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def publish_ui_info(self):
        """ 
        [STM32 main.c 파싱 로직과 동기화]
        STM32 Expects: Mode @ Speed @ Battery @ Caller @ Start @ Dest
        """
        s_mode = str(self.current_state)
        # 속도는 현재 계산 안하므로 0.0 고정
        s_speed = "0.0" 
        s_batt = str(int(self.battery_percent))
        s_caller = self.current_caller if self.current_caller else "Waiting"
        s_start = "-" # 출발지 정보는 현재 없음
        s_dest = self.current_dest_name if self.current_dest_name else "?"
        
        # 순서 중요: 모드@속도@배터리@호출자@출발지@도착지
        msg = f"{s_mode}@{s_speed}@{s_batt}@{s_caller}@{s_start}@{s_dest}"
        self.ui_pub.publish(String(data=msg))

    # ---------------------------------------------------------------------
    # TCP 통신 및 유틸리티
    # ---------------------------------------------------------------------
    def connect(self):
        if self.sock: return True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) 
            self.sock.connect((self.server_ip, self.server_port))
            self.sock.settimeout(None)
            self.logged_in = False
            print("[Net] 서버 연결 성공!")
            return True
        except: return False

    def send_login_once(self):
        if not self.logged_in and self.sock:
            try:
                pkt = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, MSG_LOGIN_REQ, len(self.robot_name)) + self.robot_name.encode()
                self.sock.sendall(pkt)
                self.logged_in = True
            except: self.close_socket()

    def send_robot_state(self):
        if not self.sock: return
        try:
            payload = struct.pack(STATE_FMT, 
                                  int(self.battery_percent), self.x, self.y, self.theta,
                                  int(self.current_state), int(self.ultra_distance), 
                                  int(1 if self.seat_detected else 0))
            header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, MSG_ROBOT_STATE, len(payload))
            self.sock.sendall(header + payload)
        except: self.close_socket()

    def rx_loop(self):
        while self.running and rclpy.ok():
            if not self.sock: time.sleep(1); continue
            try:
                hdr = self.sock.recv(HDR_SIZE)
                if len(hdr) != HDR_SIZE: self.close_socket(); continue
                
                magic, dev, mtype, dlen = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue

                payload = self.sock.recv(dlen) if dlen > 0 else b""
                
                if mtype == MSG_ASSIGN_GOAL:
                    order, sx, sy, gx, gy, rname = struct.unpack(GOAL_FMT, payload)
                    caller = rname.split(b'\x00')[0].decode('utf-8')
                    
                    print(f"\n[Server] 📩 명령 수신: Order {order}, Caller {caller}")
                    
                    self.current_caller = caller
                    self.caller_pub.publish(String(data=caller))
                    self.final_goal_x = gx
                    self.final_goal_y = gy
                    
                    if order == 6: 
                        self.mission_mode = "PICKUP"
                        self.change_state(STATE_HEADING)
                        self.start_path_navigation(sx, sy) 
                        
                    elif order in [1, 4, 5]: 
                        self.mission_mode = "MOVE"
                        self.change_state(STATE_RUNNING)
                        self.start_path_navigation(gx, gy) 
                        
            except: self.close_socket()

    def close_socket(self):
        if self.sock: 
            try: self.sock.close()
            except: pass
            self.sock = None
            self.logged_in = False
            print("[Net] 연결 끊김. 재연결 대기...")

def main():
    rclpy.init()
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "wc1"
    map_file   = sys.argv[2] if len(sys.argv) > 2 else "map_graph.json"
    node = TcpBridge(robot_name, map_file)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: 
        node.running = False
        node.close_socket()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()