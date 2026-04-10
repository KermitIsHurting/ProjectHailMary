from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cuas_fusion')
    config = os.path.join(pkg_share, 'config', 'predictor_params.yaml')
    system_params = os.path.join(pkg_share, 'config', 'system_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'cuas_demo.rviz')
    radar_config_script = os.path.join(
        pkg_share, 'scripts', 'send_radar_config.sh')
    radar_profile = os.path.join(pkg_share, 'config', 'radar_profile.cfg')

    send_radar_config = ExecuteProcess(
        cmd=['bash', '-c',
             f'{radar_config_script} {radar_profile} /dev/radar_config'
             ' && sleep 2'],
        name='send_radar_config',
        output='screen',
    )

    radar_parser_node = Node(
        package='cuas_fusion',
        executable='radar_parser_node',
        name='radar_parser_node',
        parameters=[system_params, {'use_sim_time': False}],
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_radar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_frame'],
        ),
        send_radar_config,
        RegisterEventHandler(
            OnProcessExit(
                target_action=send_radar_config,
                on_exit=[radar_parser_node],
            )
        ),
        Node(
            package='cuas_fusion',
            executable='imm_tracker_node',
            name='imm_tracker_node',
            parameters=[{'use_sim_time': False}],
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
                'engine_path': '/home/zork/ProjectHailMarry/models/yolov8s_fp16.engine',
            }],
        ),
        Node(
            package='cuas_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=[{'use_sim_time': False}],
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
            executable='cuas_visualizer_node',
            name='cuas_visualizer_node',
            parameters=[system_params, {'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ),
    ])
