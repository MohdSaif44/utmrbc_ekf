from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_odom_pub',
            namespace='imu_data',
            executable='imu_data',
            name='imu_publisher'
        ),
        Node(
            package='py_odom_pub',
            namespace='odom_data',
            executable='odom_data',
            name='odom_publisher'
        ),
        Node(
            package='py_odom_pub',
            namespace='laser_data',
            executable='laser_data',
            name='laser_publisher',
        )
    ])