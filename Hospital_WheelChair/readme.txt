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


//2512018 임정민
urdf 파일에  <mesh filename= ~~> 부분 meshs 파일 경로에 맞게 수정해주어아함

터미널 1)
ubuntu@ubuntu:~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf$  
source /opt/ros/humble/setup.bash
ubuntu@ubuntu:~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf$ 
 source ~/turtlebot3_ws/install/setup.bash
ubuntu@ubuntu:~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf$
 ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat turtlebot3_burger_Hospital_fixed.urdf)"

터미널 2)
ubuntu@ubuntu:~$ source /opt/ros/humble/setup.bash
ubuntu@ubuntu:~$ source ~/turtlebot3_ws/install/setup.bash
ubuntu@ubuntu:~$ ros2 run joint_state_publisher joint_state_publisher

터미널 3)
ubuntu@ubuntu:~$ source /opt/ros/humble/setup.bash
ubuntu@ubuntu:~$ source ~/turtlebot3_ws/install/setup.bash
ubuntu@ubuntu:~$ rviz2


display 설정 후 RViz 화면 클릭 → F 키
