import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

# 有图片编码端，解码端，串口节点
def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import Shutdown
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch_ros.descriptions import ComposableNode

    robot_params = Path(__file__).resolve().parent.parent / 'config' / 'config.yaml'
    camera_params = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')

    with robot_params.open('r', encoding='utf-8') as f:
        robot_cfg = yaml.safe_load(f) or {}

    lob_cfg = robot_cfg.get('/landing_point_detector', {}).get('ros__parameters', {})
    enable_serial_transmitter = bool(lob_cfg.get('enable_serial_transmitter', False))
    enable_decode_node        = bool(lob_cfg.get('enable_decode_node', False))

    hik_camera_node = ComposableNode(
        package='hik_camera',
        plugin='hik_camera::HikCameraNode',
        name='hik_camera',
        parameters=[camera_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    landing_point_detector_node = ComposableNode(
        package='landing_point_detector',
        plugin='landing_point_detector::LandingPointDetectorNode',
        name='landing_point_detector',
        parameters=[str(robot_params)],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    serial_transmitter_node = ComposableNode(
        package='image_serial',
        plugin='image_serial::SerialTransmitterNode',
        name='serial_transmitter_node',
        parameters=[str(robot_params)],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    ros2_packet_decode_receiver_node = Node(
        package='landing_point_detector',
        executable='ros2_packet_decode_receiver',
        name='ros2_packet_decode_receiver',
        parameters=[str(robot_params)],
        output='screen',
    )

    composable_nodes = [hik_camera_node, landing_point_detector_node]
    if enable_serial_transmitter:
        composable_nodes.append(serial_transmitter_node)

    container = ComposableNodeContainer(
        name='lob_shot_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    launch_actions = [container]

    if enable_decode_node:
        launch_actions.append(ros2_packet_decode_receiver_node)

    return LaunchDescription(launch_actions)
