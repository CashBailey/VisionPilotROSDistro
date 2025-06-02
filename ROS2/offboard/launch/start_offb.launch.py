from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_offboard',
            executable='offb_node',
            output='screen'),
        Node(
            package='ros2_offboard',
            executable='pbvs_node',
            name='pbvs_node',
            output='screen'),
    ])
