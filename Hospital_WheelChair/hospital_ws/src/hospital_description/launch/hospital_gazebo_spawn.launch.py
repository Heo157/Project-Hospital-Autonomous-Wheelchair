from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_hospital = get_package_share_directory('hospital_description')

    gazebo_launch = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')

    # hospital_description/share 를 GAZEBO_MODEL_PATH에 추가
    model_path = os.path.dirname(pkg_hospital)  # .../hospital_description (share 폴더 기준)

    map_urdf = os.path.join(pkg_hospital, 'urdf', 'map_hospital.urdf')
    bot_urdf = os.path.join(pkg_hospital, 'urdf', 'turtlebot3_burger_Hospital_fixed.urdf')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'hospital_map', '-file', map_urdf, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'hospital_bot', '-file', bot_urdf, '-x', '0.5', '-y', '-1', '-z', '0.2'],
            output='screen'
        ),
    ])
