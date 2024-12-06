from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    another_launch_file = os.path.join(
        get_package_share_directory('robot_localization'),
        'launch',
        'ekf.launch.py'
    )
    return LaunchDescription([
        Node(
            package='py_odom_pub',
            executable='laser_data',
            name='laser_publisher',
        ),
        Node(
            package='py_odom_pub',
            executable='imu_data',
            name='imu_publisher'
        ),
        Node(
            package='py_odom_pub',
            executable='odom_data',
            name='odom_publisher'
        ),
        Node(
            package='py_odom_pub',
            executable='local_data',
            name='local_publisher'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM1']
        ),  
         Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(another_launch_file)
        )
        
    ])