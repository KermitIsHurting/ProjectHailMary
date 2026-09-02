from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cuas_fusion')
    config = os.path.join(pkg_share, 'config', 'predictor_params.yaml')
    system_params = os.path.join(pkg_share, 'config', 'system_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'cuas_demo.rviz')

    # Only the sensor topics come from the bag; /tracks, /threat/reports and
    # /fusion/detections are produced live, so the graph is not fed twice
    # (RC-23). The April 2026 bags' cuas_msgs topics no longer deserialize
    # against the current package anyway (audit C-33); their
    # /radar/detections still replays.
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/demo_take3'),
        description='Path to the rosbag directory to replay (radar and camera topics only are played)',
    )

    sim_time = {'use_sim_time': True}

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             LaunchConfiguration('bag_path'),
             '--clock',
             '--topics', '/radar/detections', '/camera/image_raw'],
        name='rosbag_play',
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_frame'],
        ),
        Node(
            package='cuas_fusion',
            executable='imm_tracker_node',
            name='imm_tracker_node',
            parameters=[sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='kinematic_predictor_node',
            name='kinematic_predictor_node',
            parameters=[config, sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='occlusion_predictor_node',
            name='occlusion_predictor_node',
            parameters=[config, sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='prediction_mux_node',
            name='prediction_mux_node',
            parameters=[sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=[sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='classifier_node',
            name='classifier_node',
            parameters=[system_params, sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='cot_publisher_node',
            name='cot_publisher_node',
            parameters=[sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='cuas_visualizer_node',
            name='cuas_visualizer_node',
            parameters=[system_params, sim_time],
        ),
        Node(
            package='cuas_fusion',
            executable='cuas_overlay_node',
            name='cuas_overlay_node',
            parameters=[sim_time],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time],
        ),
        bag_play,
    ])
