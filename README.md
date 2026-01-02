# 🏥 Hospital Autonomous Wheelchair Robot  

> **요약**  
> TurtleBot3 Burger 플랫폼에 3D 프린팅 휠체어 구조물을 장착하고,  
> **LiDAR SLAM 기반 자율주행(ROS2/Nav2)** + **중앙 서버/DB 배차 시스템** + **Qt 터치 키오스크(외래 호출)** + **STM32U5(초음파/압력 + Touch-GFX UI)** 를 결합해  
> 병원 환경에서 **환자 호출 → 탑승 확인 → 목적지 이동 → 도착 알림 → 하차/대기/충전** 흐름을 구현하는 프로젝트입니다.

---

## 📌 1. 프로젝트 목표

- 병원에서 거동이 불편한 환자가 휠체어를 쉽게 호출하고 이동할 수 있도록 지원
- **외래 환자**는 QR 대신 **터치 키오스크(Qt)** 로 호출
- **입원 환자**는 DB에 등록된 병동/병실 좌표 기반으로 배차 가능
- 자율주행은 **LiDAR SLAM + Nav2** 로 수행
- **STM32U5G9J-DK2 보드**로:
  - LiDAR가 감지하기 어려운 **낮은 높이 장애물**을 초음파로 감지
  - **압력 센서(FSR)**로 환자 탑승/하차를 감지하여 서버/DB에 반영
  - 보드 내장 **Touch-GFX**에서 로봇 상태/토픽값을 실시간 UI로 표시

---

## ✨ 2. 핵심 기능 (Key Features)

### ✅ 자율주행
- SLAM으로 맵 생성
- AMCL로 위치 추정
- Nav2로 경로 계획 및 목표 지점 이동

### ✅ 외래 호출: Qt 터치 키오스크
- 외래 환자가 터치 키오스크에서 “휠체어 호출”
- 출발/목적지 선택(또는 스테이션 기준 호출)
- 호출 정보는 서버/DB에 저장되고, 배차 로직이 로봇을 할당

### ✅ 서버/DB 기반 배차(Dispatch)
- DB의 호출 큐(call queue)를 기반으로
  - 가용 로봇 탐색
  - 우선순위 배정
  - 로봇에 start/goal 좌표 전달
  - 로봇 상태를 DB에 지속 기록

### ✅ STM32U5 보드(초음파 + 압력) + Touch-GFX UI
- 초음파(HC-SR04): **하단 장착** → 낮은 장애물 감지용
- 압력(FSR): **seat_detected** → 탑승 감지용
- Touch-GFX UI에 ROS 토픽 기반 상태 표시

---

## 🧠 3. 시스템 아키텍처 (System Architecture)

### 구성 요소

#### 1) Robot (TurtleBot3 Burger+ Raspberry Pi 4)
- ROS2 (Nav2/SLAM/AMCL)
- LiDAR(`/scan`), odom(`/odom`), 배터리(`/battery_state`) publish
- 서버와 네트워크 통신(프로젝트 방식에 맞는 브리지/프로토콜)
- STM32U5 보드와 UART/Serial 등으로 연동

#### 2) STM32U5 Sensor Module
- **초음파 거리 토픽**: `/ultra_distance_cm`
- **탑승 감지 토픽**: `/seat_detected`
- Touch-GFX에 아래 토픽 UI 표시

#### 3) Central Server (Linux/Ubuntu)
- C 기반 TCP 서버(멀티프로세스)
- MariaDB/MySQL 연동(로봇 상태 UPSERT, 호출 큐 관리)
- 배차/로봇 매니저/클라이언트 핸들러 프로세스

#### 4) Qt Apps
- **Qt Admin Dashboard**: 관리자 모니터링/알림
- **Qt Touch Kiosk**: 외래 호출 UI(터치스크린)

---

## 🗺️ 4. 동작 시나리오 (Workflow)

### 4.1 외래 환자 호출(키오스크)
1. 외래 환자가 **Qt 키오스크**에서 휠체어 호출
2. 키오스크가 서버/DB에 호출 등록(call queue insert)
3. 서버 Dispatch가 가용 로봇을 선택하여 배차
4. 로봇은 출발지 → 목적지로 Nav2 goal 이동
5. 도착 시 서버/관리자 UI에 알림

### 4.2 입원 환자 호출(병동/병실 기반)
1. 입원 환자/관리자가 호출(병실/병동 선택)
2. 서버가 DB의 위치(map_location) 좌표를 참조
3. 로봇 배차 후 해당 위치로 이동

### 4.3 탑승/하차 판단(압력 센서)
- STM32U5의 FSR 압력 센서로 `seat_detected = 1/0` 판별
- Raspberry Pi4가 이를 수신하고 서버에 전달하여 DB에 기록

### 4.4 하단 장애물 안전(초음파)
- LiDAR는 보통 바닥 가까운 낮은 장애물을 놓칠 수 있음
- STM32U5 초음파를 하단에 장착하여 `/ultra_distance_cm` publish
- 임계 거리 이하면:
    -Nav2 goal cancel/재계획(설계에 따라)

---

## 📡 5. ROS2 토픽 인터페이스 (UI 표시용)

STM32U5 Touch-GFX(또는 관리자 UI)에 표시할 토픽들:

| Topic | 의미 | UI 표시 예시 |
|------|------|-------------|
| `/amcl_pose` | 맵 기준 현재 위치 | x, y, yaw |
| `/odom` | odom/속도 | v, w, 누적 |
| `/battery_state` | 배터리 상태 | %, charging |
| `/goal_pose` | 목표 좌표 | goal x, y |
| `/scan` | LiDAR 스캔 | min range |
| `/ultra_distance_cm` | 초음파 거리(cm) | min 거리 |
| `/seat_detected` | 탑승 감지(0/1) | Seated/Empty |

> `/scan`은 데이터가 크므로 UI에서는 일반적으로 **최소 거리(min range)** 같은 요약값만 표시합니다.

---

## 🧱 Database (MariaDB/MySQL)

> 본 프로젝트는 `hospital_backup.sql` 기준으로 DB를 구성합니다.  
> 서버(C 코드)는 DB를 주기적으로 조회/갱신하여 배차(Dispatch)를 수행합니다.

### ✅ 주요 테이블 요약

#### 1) `call_queue` — 호출 대기열 (키오스크/시스템 호출 저장)

| 컬럼명 | 설명 | 예시 |
|---|---|---|
| `call_id`  | 호출 ID | 101 |
| `call_time`  | 호출 생성 시간 | 2025-12-31 12:34:56 |
| `caller_name`   | 환자/호출자 이름 | "홍길동" |
| `start_loc`   | 출발지(장소명 문자열) | "키오스크" |
| `dest_loc`   | 목적지(장소명 문자열) | "정형외과" |
| `is_dispatched`  | 배차 여부 | 배차 여부(0/1)|
| `eta`  | 상태/예상 정보 | "이동중" |


#### 2) `map_location` — 장소명 → 좌표 매핑 테이블

| 컬럼명  | 설명 | 예시 |
|---|---|---|
| `location_id`  | 장소 ID | 12 |
| `location_name` | 장소명(문자열) | "진료실 2" |
| `x`   | 맵 좌표 X | 1.25 |
| `y`   | 맵 좌표 Y | -3.40 |

> 서버 배차 시 `call_queue.start_loc`, `call_queue.dest_loc`을 `map_location`에서 찾아 (x, y)로 변환합니다.


#### 3) `robot_status` — 로봇 상태 + 명령 채널(핵심 테이블)

| 구분 | 컬럼명  | 설명 |
|---|---|---|
| 식별 | `robot_id` | 로봇 고유 ID |
| 식별 | `name`  | 로봇 이름(서버가 로봇 구분에 사용) |
| 네트워크 | `ip_address`   | 로봇 IP |
| 상태 | `op_status`  | `WAITING/HEADING/BOARDING/RUNNING/STOP/ARRIVED/EXITING/CHARGING/ERROR` |
| 전원 | `battery_percent`   | 배터리 % |
| 전원 | `is_charging`   | 충전 중 여부(0/1) |
| 현재 위치 | `current_x`   | 현재 X |
| 현재 위치 | `current_y`   | 현재 Y |
| 현재 방향 | `current_theta`   | 현재 θ(yaw) |
| 시작 좌표 | `start_x`  | 배차된 출발지 X |
| 시작 좌표 | `start_y`   | 배차된 출발지 Y |
| 목표 좌표 | `goal_x`   | 배차된 목적지 X |
| 목표 좌표 | `goal_y`   | 배차된 목적지 Y |
| 센서/기타 | `sensor`   | 센서 상태/메모 |
| **명령** | **`order`**   | **명령 존재 여부(서버: `order>0`이면 로봇에게 전송)** |
| 호출자 | `who_called`   | 호출자 이름 저장 |

> `order`는 서버 → 로봇 명령 전달을 위한 “DB 기반 명령 채널” 역할입니다.


> `order`는 서버 → 로봇 명령 전달을 위한 “DB 기반 명령 채널” 역할입니다.


#### 4) `patient_info`, `disease_types` — 우선순위 배차(응급/질병/호출시간)

##### `patient_info` (환자 정보)

| 컬럼(예시) | 설명 |
|---|---|
| `name` | 환자 이름 |
| `disease_code` | 질병 코드 |
| `is_emergency` | 응급 여부(1이면 최우선) |
| `type` | 환자 타입(OUT/IN) |

##### `disease_types` (질병 우선순위 기준)

| 컬럼(예시) | 설명 |
|---|---|
| `base_priority` | 우선순위 점수(클수록 우선) |

✅ **배차 우선순위 정렬**

| 우선순위 | 기준 | 정렬 방향 |
|---|---|---|
| 1 | `is_emergency` | DESC (응급 먼저) |
| 2 | `base_priority` | DESC (점수 높은 질병 먼저) |
| 3 | `call_time` | ASC (먼저 온 호출 먼저) |

---

## 🔧 7. 하드웨어 구성 (Hardware)

### ✅ Robot Side
- TurtleBot3 Burger base + LiDAR
- Raspberry Pi 4
- OpenCR

### ✅ STM32U5 Sensor Module
- STM32U5 보드(내장 Touch-GFX)
- IUM-100 초음파 센서 (하단 장애물 감지)
- FSR 압력 센서 (탑승 감지)
- E-STOP (응급정지 버튼)

---

## 🧰 8. 기술 스택 (Tech Stack)

- **Robot**: ROS2, Nav2, SLAM, AMCL
- **Edge**: Raspberry Pi 4
- **Sensor Board**: STM32U5G9J-DK2 + Touch-GFX UI
- **Server**: C Socket Server(Multi-process), MariaDB/MySQL
- **UI**: Qt (Admin Dashboard / Touch Kiosk)

---

## 👥 팀원 소개 (Team)

| &nbsp;&nbsp;&nbsp;&nbsp;이름&nbsp;&nbsp;&nbsp;&nbsp; | &nbsp;&nbsp;&nbsp;&nbsp;역할&nbsp;&nbsp;&nbsp;&nbsp; | 파트 | 담당 기능(요약) | 사용 기술/도구 |
| :---: | :---: | :--- | :--- | :--- |
| **허진경** | 팀장 | Robot / ROS | TurtleBot3 자율주행(ROS2/Nav2/SLAM) 통합,<br>STM32U5(초음파/압력) 센서 모듈 연동,<br>ROS 토픽 → Touch-GFX UI 표시 | ROS2, Nav2, SLAM,<br>STM32 HAL, UART,<br>Raspberry Pi 4 |
| **강송구** | 부팀장 | Server / DB | C 서버 구현, MariaDB 스키마/쿼리 설계,<br>로봇 상태 저장/배차 로직 | C(Socket), SQL,<br>MariaDB / MySQL |
| **김선곤** | 팀원 | Qt Kiosk /<br>STM32U5 Touch-GFX | 외래 환자용 터치 키오스크(Qt) UI 구현,<br>STM32U5 Touch-GFX UI 연동(옵션) | Qt,  |
| **임정민** | 팀원 | DB / ROS | DB 데이터 관리/정리, URDF | SQL,<br>MariaDB / MySQL /ROS2 |
| **유종민** | 팀원 | Firmware /<br>ROS | STM32U5(초음파/압력) 센서 구현 및<br>ROS2 연동, 3D 프린팅 구조물 제작 | ROS2, STM32,<br>Fusion 360 |






