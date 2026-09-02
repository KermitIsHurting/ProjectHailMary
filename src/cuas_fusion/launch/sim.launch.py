# WHY: replaces hardware radar with SimRadar for software-in-the-loop testing
# so the full pipeline can run without an IWR6843ISK attached.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cuas_fusion')
    config = os.path.join(pkg_share, 'config', 'predictor_params.yaml')
    system_params = os.path.join(pkg_share, 'config', 'system_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'cuas_demo.rviz')
    geofence_config = os.path.join(pkg_share, 'config', 'geofence_zones.yaml')

    default_engine_path = os.path.join(
        os.path.expanduser('~'), 'ProjectHailMarry', 'models',
        'yolov8s_int8.engine')
    default_extrinsics = os.path.join(pkg_share, 'config', 'extrinsics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_path',
            default_value=default_engine_path,
            description='TensorRT engine file for inference_node.',
        ),
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=default_extrinsics,
            description='Radar-to-camera SE(3) extrinsics parameter file '
                        '(see config/extrinsics.yaml).',
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='approach',
            description='sim_radar_node scene: approach | lateral | circle | '
                        'two_targets | crossing | clutter | max_range | hover',
        ),
        DeclareLaunchArgument(
            'noise_seed',
            default_value='42',
            description='sim_radar_node noise seed',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_frame'],
        ),
        Node(
            package='cuas_fusion',
            executable='sim_radar_node',
            name='sim_radar_node',
            parameters=[{
                'use_sim_time': False,
                'scenario': LaunchConfiguration('scenario'),
                'publish_rate_hz': 20.0,
                'noise_seed': LaunchConfiguration('noise_seed'),
            }],
        ),
        Node(
            package='cuas_fusion',
            executable='clutter_map_node',
            name='clutter_map_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='imm_tracker_node',
            name='imm_tracker_node',
            parameters=[{'use_sim_time': False}],
            remappings=[('/radar/detections', '/radar/filtered')],
        ),
        Node(
            package='cuas_fusion',
            executable='kinematic_predictor_node',
            name='kinematic_predictor_node',
            parameters=[config, {'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='occlusion_predictor_node',
            name='occlusion_predictor_node',
            parameters=[config, {'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='prediction_mux_node',
            name='prediction_mux_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='camera_node',
            name='camera_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='inference_node',
            name='inference_node',
            parameters=[{
                'use_sim_time': False,
                'engine_path': LaunchConfiguration('engine_path'),
            }],
        ),
        Node(
            package='cuas_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=[LaunchConfiguration('extrinsics_file'),
                        {'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='classifier_node',
            name='classifier_node',
            parameters=[system_params, {'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='cot_publisher_node',
            name='cot_publisher_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='geofence_node',
            name='geofence_node',
            parameters=[{
                'use_sim_time': False,
                'geofence_config_path': geofence_config,
                'publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='cuas_fusion',
            executable='reachability_node',
            name='reachability_node',
            parameters=[{
                'use_sim_time': False,
                'min_threat_level': 1,
                'publish_rate_hz': 20.0,
            }],
        ),
        Node(
            package='cuas_fusion',
            executable='health_monitor_node',
            name='health_monitor_node',
            parameters=[{'use_sim_time': False, 'publish_rate_hz': 1.0}],
        ),
        Node(
            package='cuas_fusion',
            executable='intent_classifier_node',
            name='intent_classifier_node',
            parameters=[{'use_sim_time': False, 'publish_rate_hz': 10.0}],
        ),
        Node(
            package='cuas_fusion',
            executable='cuas_visualizer_node',
            name='cuas_visualizer_node',
            parameters=[system_params, {'use_sim_time': False}],
        ),
        Node(
            package='cuas_fusion',
            executable='cuas_overlay_node',
            name='cuas_overlay_node',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ),
    ])
