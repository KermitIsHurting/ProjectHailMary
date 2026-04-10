from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cuas_fusion'), 'config', 'predictor_params.yaml')

    return LaunchDescription([
        Node(
            package='cuas_fusion',
            executable='radar_parser_node',
            name='radar_parser_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='imm_tracker_node',
            name='imm_tracker_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='occlusion_predictor_node',
            name='occlusion_predictor_node',
            parameters=[config, {'use_sim_time': False}],
        ),
    ])
