#!/usr/bin/env python3
"""
RealSense D435i source launch file.

Launches the head D435i camera driver and the generic stream source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
import yaml


def _default_head_serial() -> str:
    config_path = Path.home() / ".openflex" / "cameras_config.yaml"
    if not config_path.exists():
        return ""
    try:
        with config_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return str(data.get("cameras", {}).get("head", {}).get("serial_no", "")).strip()
    except Exception:
        return ""


def _serial_param(context) -> str:
    serial_number = LaunchConfiguration('serial_number').perform(context).strip()
    if not serial_number:
        return ''
    return serial_number if serial_number.startswith('_') else f'_{serial_number}'


def launch_setup(context):
    color_width = int(LaunchConfiguration('color_width').perform(context))
    color_height = int(LaunchConfiguration('color_height').perform(context))
    color_fps = int(LaunchConfiguration('color_fps').perform(context))
    depth_width = int(LaunchConfiguration('depth_width').perform(context))
    depth_height = int(LaunchConfiguration('depth_height').perform(context))
    depth_fps = int(LaunchConfiguration('depth_fps').perform(context))

    color_profile = f'{color_width}x{color_height}x{color_fps}'
    depth_profile = f'{depth_width}x{depth_height}x{depth_fps}'

    realsense_driver = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'serial_no': _serial_param(context),
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
            'enable_color': True,
            'enable_depth': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
            'align_depth.enable': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': ParameterValue(LaunchConfiguration('enable_imu'), value_type=bool),
            'enable_accel': ParameterValue(LaunchConfiguration('enable_imu'), value_type=bool),
            'pointcloud.enable': False,
        }],
        remappings=[
            ('/camera/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/camera/color/image_rect_raw', '/camera/color/image_raw'),
            ('/camera/camera/color/camera_info', '/camera/color/camera_info'),
            ('/camera/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_raw'),
            ('/camera/camera/aligned_depth_to_color/camera_info', '/camera/depth/camera_info'),
        ],
        output='screen',
    )

    source_node = Node(
        package='openarmx_head_vision_h264',
        executable='camera_stream_source',
        name='camera_stream_source',
        parameters=[{
            'enable_depth': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
            'color_input_topic': LaunchConfiguration('color_input_topic'),
            'color_camera_info_input_topic': LaunchConfiguration(
                'color_camera_info_input_topic'
            ),
            'depth_input_topic': LaunchConfiguration('depth_input_topic'),
            'depth_camera_info_input_topic': LaunchConfiguration(
                'depth_camera_info_input_topic'
            ),
            'color_output_topic': '/vision/color/image_raw',
            'color_camera_info_output_topic': '/vision/color/camera_info',
            'depth_output_topic': '/vision/depth/image_raw',
            'depth_camera_info_output_topic': '/vision/depth/camera_info',
            'image_reliability': 'reliable',
            'enable_stats': True,
            'stats_interval': 5.0,
        }],
        output='screen',
    )

    return [realsense_driver, source_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_number',
            default_value=_default_head_serial(),
            description='Head RealSense D435i serial number; defaults to ~/.openflex/cameras_config.yaml cameras.head.serial_no; empty means auto-select'
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value='640',
            description='Color stream width'
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='480',
            description='Color stream height'
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='30',
            description='Color stream FPS'
        ),
        DeclareLaunchArgument(
            'color_input_topic',
            default_value='/camera/color/image_raw',
            description='Upstream color image topic from RealSense D435i driver'
        ),
        DeclareLaunchArgument(
            'color_camera_info_input_topic',
            default_value='/camera/color/camera_info',
            description='Upstream color camera info topic from RealSense D435i driver'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Enable depth in both D435i driver and source node'
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Enable D435i gyro and accelerometer streams'
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value='640',
            description='Depth stream width'
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value='480',
            description='Depth stream height'
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value='15',
            description='Depth stream FPS'
        ),
        DeclareLaunchArgument(
            'depth_input_topic',
            default_value='/camera/depth/image_raw',
            description='Upstream aligned depth image topic from RealSense D435i driver'
        ),
        DeclareLaunchArgument(
            'depth_camera_info_input_topic',
            default_value='/camera/depth/camera_info',
            description='Upstream aligned depth camera info topic from RealSense D435i driver'
        ),
        OpaqueFunction(function=launch_setup),
    ])
