#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================================
 파일명: tcp_bridge.py (Final Release Version)
 설명:   ROS 2(Nav2) <-> TCP(C Server) 통신 브리지 및 로봇 제어기
 
 [사용자 요청 반영 사항]
 1. [Nav] 도착 판정 범위 완화: 0.2m -> 0.5m (DIST_TOLERANCE_FINAL 수정)
 2. [Logic] 강제 상태 전환 로직 제거: 도착 시 자동 초기화 안 함 (버튼 대기)
 3. [UI] 목적지 이름 표시: map_graph.json의 locations 정보를 매핑하여 '?' 해결
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
MSG_LOGIN_REQ   = 0x01  # 로그인 (이름 전송)
MSG_ROBOT_STATE = 0x20  # 로봇 상태 보고 (주기적)
MSG_ASSIGN_GOAL = 0x30  # 작업 지시 (서버 -> 로봇)

# 로봇 상태 코드 (FSM)
STATE_WAITING  = 0  # 대기 중 (IDLE)
STATE_HEADING  = 1  # 호출자(환자)에게 이동 중
STATE_BOARDING = 2  # 도착 후 탑승 대기
STATE_RUNNING  = 3  # 목적지로 주행 중
STATE_STOP     = 4  # 비상 정지
STATE_ARRIVED  = 5  # 목적지 도착 (하차 대기)
STATE_EXITING  = 6  # 하차 완료 처리 중 (사용 안 함, 예비)
STATE_CHARGING = 7  # 충전 중
STATE_ERROR    = 99 # 에러

# 네트워크 패킷 포맷 (struct.pack/unpack 용)
HDR_FMT = "<BBBB"   # 헤더: Magic(1)+Device(1)+Type(1)+Len(1)
HDR_SIZE = struct.calcsize(HDR_FMT)
STATE_FMT = "<ifffBiB" # 본문: 배터리,x,y,theta,상태,초음파,착석
GOAL_FMT = "<iffff64s" # 본문: 명령코드,시작x,y,목표x,y,호출자이름
GOAL_SIZE = struct.calcsize(GOAL_FMT)

# 하드웨어 버튼 매핑값 (STM32 -> ROS Topic)
BTN_BOARDING_COMPLETE = 1  # 탑승 완료 (출발)
BTN_RESUME            = 3  # 주행 재개
BTN_EMERGENCY         = 4  # 비상 정지
BTN_EXIT_COMPLETE     = 5  # 하차 완료 (복귀)

# [설정] 주행 허용 오차 (단위: 미터)
# 0.5m 이내에 들어오면 도착한 것으로 간주합니다.
DIST_TOLERANCE_FINAL    = 0.5  
# 경유지는 1.0m 근처만 가도 다음 지점으로 넘어갑니다 (부드러운 주행).
DIST_TOLERANCE_WAYPOINT = 1.0  

# =========================================================================
# 2. 길찾기 및 맵 데이터 관리 클래스
# =========================================================================
class SimplePathFinder:
    """
    map_graph.json 파일을 읽어서 다음 기능을 수행합니다.
    1. A* 알고리즘을 통한 최단 경로 탐색 (Nodes & Edges)
    2. 좌표(x, y)를 사람이 읽을 수 있는 장소 이름으로 변환 (Locations)
    """
    def __init__(self, json_path):
        self.nodes = {}      # 노드 ID -> (x, y)
        self.edges = {}      # 노드 연결 정보
        self.locations = {}  # 장소 이름 -> (x, y)
        self.load_map(json_path)

    def load_map(self, json_path):
        """ JSON 파일을 파싱하여 메모리에 적재 """
        print(f"[Map] 맵 파일 로딩 시작: {json_path}")
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 1. 노드 정보 로드
            self.nodes = {int(k): tuple(v) for k, v in data.get('nodes', {}).items()}
            
            # 2. 간선 정보 로드
            self.edges = {}
            for item in data.get('edges', []):
                if len(item) >= 3:
                    u, v, w = item[0], item[1], item[2]
                    self.edges.setdefault(u, []).append((v, w))
                    self.edges.setdefault(v, []).append((u, w))

            # 3. [중요] 장소 이름 정보 로드 (Locations)
            # 이 부분이 있어야 STM32 화면의 '?'가 실제 이름으로 표시됩니다.
            raw_locs = data.get('locations', {})
            for name, coords in raw_locs.items():
                self.locations[name] = tuple(coords)

            print(f"[Map] 로딩 완료: 노드 {len(self.nodes)}개, 정의된 장소 {len(self.locations)}개")
            
        except Exception as e:
            print(f"[Map] ⚠️ 맵 로딩 실패: {e}")
            # 실패 시 빈 딕셔너리로 초기화하여 크래시 방지
            self.nodes = {}
            self.edges = {}
            self.locations = {}

    def find_location_name(self, target_x, target_y):
        """ 
        주어진 좌표(target_x, target_y)와 가장 가까운 장소 이름을 찾습니다.
        STM32 UI 표시용으로 사용됩니다.
        """
        if not self.locations:
            return "?"
        
        # 가장 가까운 장소를 찾기 위해 거리 비교 (검색 반경 1.5m 이내)
        min_dist = 1.5
        found_name = "?"
        
        for name, coords in self.locations.items():
            dist = math.dist((target_x, target_y), coords)
            if dist < min_dist:
                min_dist = dist
                found_name = name
                
        return found_name

    def find_nearest_node(self, tx, ty):
        """ 좌표에서 가장 가까운 노드 ID를 찾습니다. """
        if not self.nodes: return None
        return min(self.nodes.keys(), key=lambda k: math.dist((tx, ty), self.nodes[k]))

    def get_path(self, sx, sy, gx, gy):
        """ 시작점(sx,sy)에서 목표점(gx,gy)까지의 A* 경로를 반환합니다. """
        # 맵 데이터가 없으면 직선 경로 반환
        if not self.nodes: return [(gx, gy)]
        
        start_node = self.find_nearest_node(sx, sy)
        end_node = self.find_nearest_node(gx, gy)
        
        if start_node is None or end_node is None: return [(gx, gy)]
        
        # A* 알고리즘 수행
        queue = [(0, start_node, [])]
        visited = set()
        
        while queue:
            (cost, curr, path) = heapq.heappop(queue)
            if curr in visited: continue
            visited.add(curr)
            
            new_path = path + [curr]
            
            if curr == end_node:
                # 노드 ID 리스트를 좌표 리스트로 변환 후 최종 목적지 추가
                return [self.nodes[n] for n in new_path] + [(gx, gy)]
            
            for neighbor, weight in self.edges.get(curr, []):
                if neighbor not in visited:
                    # 휴리스틱: 유클리드 거리
                    h = math.dist(self.nodes[neighbor], self.nodes[end_node])
                    heapq.heappush(queue, (cost + weight + h, neighbor, new_path))
        
        # 경로 못 찾으면 직선 이동
        return [(gx, gy)]

# =========================================================================
# 3. 메인 ROS 2 노드 클래스
# =========================================================================
class TcpBridge(Node):
    def __init__(self, robot_name_arg, map_file_arg):
        super().__init__("tcp_bridge")
        
        # 3.1 변수 초기화
        self.robot_name = robot_name_arg
        self.server_ip = self.declare_parameter("server_ip", "127.0.0.1").value
        self.server_port = 8080
        
        # 로봇 상태 정보
        self.current_state = STATE_WAITING
        self.mission_mode = "NONE" # PICKUP(픽업), DELIVER(이송), MOVE(단순이동)
        
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
        self.waypoint_queue = [] # 경유지 큐
        
        # UI 표시 정보
        self.current_caller = ""     # 호출자 이름 (예: 김철수)
        self.current_dest_name = "?" # 목적지 장소명 (예: 정형외과)
        
        # TCP 소켓
        self.sock = None
        self.lock = threading.Lock()
        self.logged_in = False
        self.running = True
        
        # 맵 로더 인스턴스 생성
        self.pathfinder = SimplePathFinder(map_file_arg)

        # 3.2 ROS Publisher / Subscriber 설정
        prefix = f"/{self.robot_name}"
        
        # 구독 (센서 -> 로봇)
        self.create_subscription(Odometry, f"{prefix}/odom", self.odom_cb, 10)
        self.create_subscription(BatteryState, f"{prefix}/battery_state", self.batt_cb, 10)
        self.create_subscription(Float32, f"{prefix}/ultra_distance_cm", self.ultra_cb, 10)
        self.create_subscription(Bool, f"{prefix}/seat_detected", self.seat_cb, 10)
        self.create_subscription(Int32, f"{prefix}/stm32/button", self.button_cb, 10)
        
        # 발행 (로봇 -> UI / Nav2)
        self.ui_pub = self.create_publisher(String, f"{prefix}/ui/info", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.caller_pub = self.create_publisher(String, f"{prefix}/caller_name", 10)
        
        # 3.3 타이머 및 스레드 시작
        # 0.5초 주기로 제어 루프 실행 (상태 체크, 서버 보고)
        self.create_timer(0.5, self.control_loop)
        
        # TCP 수신은 블로킹 되므로 별도 스레드 사용
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        
        print(f"\n🚀 [System] {self.robot_name} 시스템 시작됨.")
        print(f"   - 도착 판정 오차: {DIST_TOLERANCE_FINAL}m")
        print(f"   - 서버 주소: {self.server_ip}:{self.server_port}")

    # ---------------------------------------------------------------------
    # 콜백 함수 (데이터 수신)
    # ---------------------------------------------------------------------
    def odom_cb(self, msg):
        """ 로봇의 현재 위치(x,y) 업데이트 """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Theta 변환은 필요 시 추가 (현재 로직엔 거리만 사용)
        
    def batt_cb(self, msg):
        """ 배터리 잔량 업데이트 """
        val = int(msg.percentage) if msg.percentage > 1.0 else int(msg.percentage * 100)
        self.battery_percent = val
        
    def ultra_cb(self, msg): self.ultra_distance = int(msg.data)
    def seat_cb(self, msg): self.seat_detected = msg.data

    def button_cb(self, msg):
        """
        물리 버튼 입력 처리 핸들러
        각 상태별로 버튼의 역할이 다릅니다.
        """
        btn = msg.data
        if btn == 0: return # 노이즈 필터링
        print(f"\n[Button] 🔘 버튼 입력 감지: {btn}")
        
        # 1. [탑승 대기] 상태에서 [탑승 완료] 버튼 클릭
        if self.current_state == STATE_BOARDING and btn == BTN_BOARDING_COMPLETE:
            print("✅ 탑승 완료 확인. 목적지로 출발합니다.")
            self.change_state(STATE_RUNNING)
            # 저장해둔 최종 목적지로 이동 시작
            self.start_path_navigation(self.final_goal_x, self.final_goal_y)
            
        # 2. [도착 완료] 상태에서 [하차 완료] 버튼 클릭 (임무 종료)
        elif self.current_state == STATE_ARRIVED and btn == BTN_EXIT_COMPLETE:
            print("✅ 임무 종료. 대기 모드로 복귀합니다.")
            self.reset_to_waiting()

        # 3. [비상 정지] 버튼 (언제든지 작동)
        elif btn == BTN_EMERGENCY:
            print("🚨 비상 정지 명령!")
            self.change_state(STATE_STOP)
            self.publish_nav2_goal(self.x, self.y) # 현재 위치로 목표 설정하여 정지

        # 4. [정지 상태]에서 [재개] 버튼
        elif self.current_state == STATE_STOP and btn == BTN_RESUME:
            print("▶️ 주행 재개.")
            self.change_state(STATE_RUNNING) # 이전 상태 복구가 정석이나, 편의상 RUNNING으로
            self.publish_nav2_goal(self.current_goal_x, self.current_goal_y)

    # ---------------------------------------------------------------------
    # 핵심 로직 (State Machine & Control)
    # ---------------------------------------------------------------------
    def change_state(self, new_state):
        """ 상태 변경 및 로그 출력, UI 갱신 """
        if self.current_state != new_state:
            print(f"[State] 🔄 상태 전환: {self.current_state} -> {new_state}")
            self.current_state = new_state
        self.publish_ui_info()

    def control_loop(self):
        """ 메인 제어 루프 (0.5초마다 실행) """
        # 1. 서버 연결 관리
        if not self.connect(): return
        
        # 2. 로그인 (최초 1회)
        self.send_login_once()
        
        # 3. UI 및 서버 보고
        self.publish_ui_info()
        self.send_robot_state()

        # 4. 주행 모니터링 및 도착 판정
        if self.current_state in [STATE_HEADING, STATE_RUNNING]:
            # 남은 거리 계산
            dist = math.dist((self.x, self.y), (self.current_goal_x, self.current_goal_y))
            
            # 현재 목표가 '경유지'인지 '최종 목적지'인지 판단
            is_intermediate = (len(self.waypoint_queue) > 0)
            
            # 허용 오차 설정 (경유지는 1.0m, 최종 목적지는 0.5m)
            tolerance = DIST_TOLERANCE_WAYPOINT if is_intermediate else DIST_TOLERANCE_FINAL
            
            # [도착 판정]
            if dist < tolerance:
                if is_intermediate:
                    # 경유지 도착 -> 다음 지점으로 계속 이동
                    self.pop_and_drive() 
                else:
                    # 최종 목적지 도착!
                    print(f"[Nav] 🏁 최종 목적지 도착 완료! (오차: {dist:.2f}m)")
                    self.handle_arrival()

    def handle_arrival(self):
        """ 
        [목적지 도착 시 처리]
        사용자 요청에 따라 강제 상태 전환(자동 초기화) 로직을 제거했습니다.
        로봇은 멈추고 버튼 입력을 기다립니다.
        """
        # 1. 로봇 정지 (현재 위치를 목표로 재전송)
        self.publish_nav2_goal(self.x, self.y)

        # 2. 상태별 분기 처리
        if self.current_state == STATE_HEADING:
            # 환자 위치 도착 -> '탑승 대기' 상태로 전환
            print("[Event] 픽업 위치 도착. 환자 탑승을 대기합니다.")
            self.change_state(STATE_BOARDING)
            
        elif self.current_state == STATE_RUNNING:
            # 목적지 도착 -> '도착 완료' 상태로 전환
            print("[Event] 목적지 도착. 하차 또는 완료 버튼을 대기합니다.")
            self.change_state(STATE_ARRIVED)
            
            # [중요] 여기에 reset_to_waiting()을 넣지 않음으로써
            # 사용자가 버튼을 누르기 전까지 로봇은 계속 ARRIVED 상태를 유지함.

    def reset_to_waiting(self):
        """ 모든 미션 초기화 및 대기 상태 복귀 """
        self.change_state(STATE_WAITING)
        self.mission_mode = "NONE"
        self.current_caller = ""
        self.current_dest_name = "?"
        self.waypoint_queue = []
        print("[Logic] 시스템 대기 상태로 초기화됨.")

    # ---------------------------------------------------------------------
    # 네비게이션 헬퍼
    # ---------------------------------------------------------------------
    def start_path_navigation(self, tx, ty):
        """ 경로 생성 및 주행 시작 """
        # 1. 경로 계획 (A*)
        path = self.pathfinder.get_path(self.x, self.y, tx, ty)
        self.waypoint_queue = path
        
        # 2. 목적지 이름 찾기 (UI 표시용) - 여기가 '?' 해결의 핵심
        self.current_dest_name = self.pathfinder.find_location_name(tx, ty)
        print(f"[Nav] 새로운 경로 시작: {self.current_dest_name} ({tx:.1f}, {ty:.1f})")
        
        # 3. 주행 시작
        self.pop_and_drive()

    def pop_and_drive(self):
        """ 큐에서 다음 웨이포인트를 꺼내 Nav2로 전송 """
        if self.waypoint_queue:
            wp = self.waypoint_queue.pop(0)
            self.publish_nav2_goal(wp[0], wp[1])
            # print(f"[Nav] 이동 중... 남은 경유지: {len(self.waypoint_queue)}개")

    def publish_nav2_goal(self, x, y):
        """ ROS 2 Goal Topic 발행 """
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
        STM32 UI 표시용 메시지 발행 
        Format: 상태@호출자@목적지이름@-(예비)
        """
        s_mode = str(self.current_state)
        s_caller = self.current_caller if self.current_caller else "Waiting"
        # 여기서 find_location_name으로 찾은 이름을 보냅니다.
        s_dest = self.current_dest_name if self.current_dest_name else "?"
        
        # 4번째 필드는 현재 사용하지 않으므로 '-'로 고정 전송 (프로토콜 유지용)
        msg = f"{s_mode}@{s_caller}@{s_dest}@-"
        self.ui_pub.publish(String(data=msg))

    # ---------------------------------------------------------------------
    # TCP 통신 및 유틸리티
    # ---------------------------------------------------------------------
    def connect(self):
        """ 서버 연결 시도 """
        if self.sock: return True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) # 타임아웃 3초
            self.sock.connect((self.server_ip, self.server_port))
            self.sock.settimeout(None)
            self.logged_in = False
            print("[Net] 서버 연결 성공!")
            return True
        except: 
            return False

    def send_login_once(self):
        """ 최초 연결 시 ID 전송 """
        if not self.logged_in and self.sock:
            try:
                # 로그인 패킷: Magic + Device + Type(LOGIN) + Len + Payload(Name)
                pkt = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, MSG_LOGIN_REQ, len(self.robot_name)) + self.robot_name.encode()
                self.sock.sendall(pkt)
                self.logged_in = True
            except: self.close_socket()

    def send_robot_state(self):
        """ 서버로 로봇 상태 정보(Heartbeat) 전송 """
        if not self.sock: return
        try:
            # Payload 패킹
            payload = struct.pack(STATE_FMT, 
                                  int(self.battery_percent), self.x, self.y, self.theta,
                                  int(self.current_state), int(self.ultra_distance), 
                                  int(1 if self.seat_detected else 0))
            
            # Header + Payload 전송
            header = struct.pack(HDR_FMT, MAGIC_NUMBER, DEVICE_ROBOT_ROS, MSG_ROBOT_STATE, len(payload))
            self.sock.sendall(header + payload)
        except: self.close_socket()

    def rx_loop(self):
        """ 수신 전용 스레드 """
        while self.running and rclpy.ok():
            if not self.sock: 
                time.sleep(1); continue
            try:
                # 1. 헤더 수신
                hdr = self.sock.recv(HDR_SIZE)
                if len(hdr) != HDR_SIZE: self.close_socket(); continue
                
                magic, dev, mtype, dlen = struct.unpack(HDR_FMT, hdr)
                if magic != MAGIC_NUMBER: continue

                # 2. 데이터 수신
                payload = self.sock.recv(dlen) if dlen > 0 else b""
                
                # 3. 메시지 처리 (작업 지시)
                if mtype == MSG_ASSIGN_GOAL:
                    order, sx, sy, gx, gy, rname = struct.unpack(GOAL_FMT, payload)
                    caller = rname.split(b'\x00')[0].decode('utf-8')
                    
                    print(f"\n[Server] 📩 명령 수신: Order {order}, Caller {caller}")
                    print(f"         좌표: ({sx},{sy}) -> ({gx},{gy})")
                    
                    self.current_caller = caller
                    self.caller_pub.publish(String(data=caller))
                    self.final_goal_x = gx
                    self.final_goal_y = gy
                    
                    # Order 6: 환자 픽업 미션
                    if order == 6: 
                        self.mission_mode = "PICKUP"
                        self.change_state(STATE_HEADING)
                        self.start_path_navigation(sx, sy) # 픽업지로 이동
                        
                    # Order 1,4,5: 단순 이동/복귀 미션
                    elif order in [1, 4, 5]: 
                        self.mission_mode = "MOVE"
                        self.change_state(STATE_RUNNING)
                        self.start_path_navigation(gx, gy) # 목적지로 바로 이동
                        
            except Exception as e:
                # print(f"[Net] 수신 에러: {e}")
                self.close_socket()

    def close_socket(self):
        """ 소켓 정리 """
        if self.sock: 
            try: self.sock.close()
            except: pass
            self.sock = None
            self.logged_in = False
            print("[Net] 연결 끊김. 재연결 대기...")

# =========================================================================
# 메인 실행부
# =========================================================================
def main():
    rclpy.init()
    
    # 실행 인자 파싱 (예: python3 tcp_bridge.py wc1 map.json)
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "wc1"
    map_file   = sys.argv[2] if len(sys.argv) > 2 else "map_graph.json"
    
    node = TcpBridge(robot_name, map_file)
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.running = False
        node.close_socket()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()