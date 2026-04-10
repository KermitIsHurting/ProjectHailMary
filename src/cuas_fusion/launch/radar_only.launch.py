from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cuas_fusion',
            executable='radar_parser_node',
            name='radar_parser_node',
            parameters=[{'use_sim_time': False}],
        ),
    ])
