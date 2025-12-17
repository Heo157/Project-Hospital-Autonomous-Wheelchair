-- 작업 공간
cd ~/turtlebot3_ws/src

-- tcp_bridge 패키지 생성
ros2 pkg create wc_server_bridge --build-type ament_python --dependencies rclpy

-- 노드코드
vi wc_server_bridge/wc_server_bridge/tcp_bridge.py

-- setup.py에 entry_points 등록
entry_points={
    'console_scripts': [
        'tcp_bridge = wc_server_bridge.tcp_bridge:main',
    ],
},

-- 빌드/실행
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 run wc_server_bridge tcp_bridge
