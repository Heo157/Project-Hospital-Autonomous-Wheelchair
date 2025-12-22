from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace   = LaunchConfiguration('namespace')
    server_ip   = LaunchConfiguration('server_ip')
    server_port = LaunchConfiguration('server_port')
    robot_name  = LaunchConfiguration('robot_name')
    tx_hz       = LaunchConfiguration('tx_hz')
    use_amcl    = LaunchConfiguration('use_amcl_pose')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='wc1'),
        DeclareLaunchArgument('server_ip', default_value='10.10.14.138'),
        DeclareLaunchArgument('server_port', default_value='8080'),
        DeclareLaunchArgument('robot_name', default_value='wc1'),
        DeclareLaunchArgument('tx_hz', default_value='2.0'),
        DeclareLaunchArgument('use_amcl_pose', default_value='true'),

        GroupAction([
            # /wc1 네임스페이스로 묶고 싶으면 사용
            PushRosNamespace(namespace),

            Node(
                package='wc_server_bridge',
                executable='tcp_bridge',   # tcp_bridge 패키지의 console_scripts 이름
                name='tcp_bridge',
                output='screen',
                parameters=[{
                    'server_ip': server_ip,
                    'server_port': server_port,
                    'robot_name': robot_name,
                    'tx_hz': tx_hz,
                    'use_amcl_pose': use_amcl,
                }],
                # TF 네임스페이스 꼬임 방지(필요 시)
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
        ]),
    ])
