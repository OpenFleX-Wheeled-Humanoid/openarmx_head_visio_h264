#!/usr/bin/env python3
"""
RealSense D435i VR launch file.

Launches the head D435i camera driver, the generic stream source and the VR
adapter.
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_mode',
            default_value='low_latency',
            description='Configuration preset: balanced, high_quality, low_latency, bandwidth_saving, custom'
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value=_default_head_serial(),
            description='Head RealSense D435i serial number; defaults to ~/.openflex/cameras_config.yaml cameras.head.serial_no; empty means auto-select'
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value='640',
            description='Color stream width, used only in custom mode'
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='480',
            description='Color stream height, used only in custom mode'
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='30',
            description='Color stream FPS, used only in custom mode'
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
        DeclareLaunchArgument(
            'vr_fps',
            default_value='20',
            description='Target FPS for VR streaming'
        ),
        DeclareLaunchArgument(
            'video_bitrate_kbps',
            default_value='4000',
            description='Target video bitrate in kbps'
        ),
        DeclareLaunchArgument(
            'keyframe_interval',
            default_value='30',
            description='Video keyframe interval in frames'
        ),
        DeclareLaunchArgument(
            'video_codec',
            default_value='h264',
            description='Video codec for OAR3 streaming: h264 or hevc'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='-1',
            description='UDP port for OAR3 VR streaming; -1 uses tcp_port compatibility alias'
        ),
        DeclareLaunchArgument(
            'udp_host',
            default_value='',
            description='UDP bind host for OAR3 VR streaming; empty uses tcp_host compatibility alias'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='5600',
            description='Deprecated compatibility alias for udp_port'
        ),
        DeclareLaunchArgument(
            'tcp_host',
            default_value='0.0.0.0',
            description='Deprecated compatibility alias for udp_host'
        ),
        DeclareLaunchArgument(
            'flip_vertical',
            default_value='true',
            description='Flip the host-side image vertically before video encoding'
        ),
        DeclareLaunchArgument(
            'flip_horizontal',
            default_value='false',
            description='Flip the host-side image horizontally before video encoding'
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    config_mode = LaunchConfiguration('config_mode').perform(context)

    presets = {
        'balanced': {
            'color_width': 1280, 'color_height': 720, 'color_fps': 30,
            'vr_fps': 20, 'video_bitrate_kbps': 4000
        },
        'high_quality': {
            'color_width': 1280, 'color_height': 720, 'color_fps': 30,
            'vr_fps': 25, 'video_bitrate_kbps': 6000
        },
        'low_latency': {
            'color_width': 640, 'color_height': 480, 'color_fps': 30,
            'vr_fps': 25, 'video_bitrate_kbps': 2500
        },
        'bandwidth_saving': {
            'color_width': 640, 'color_height': 480, 'color_fps': 15,
            'vr_fps': 15, 'video_bitrate_kbps': 1500
        },
    }

    if config_mode in presets:
        preset = presets[config_mode]
        color_width = preset['color_width']
        color_height = preset['color_height']
        color_fps = preset['color_fps']
        vr_fps = preset['vr_fps']
        video_bitrate_kbps = preset['video_bitrate_kbps']
    else:
        color_width = int(LaunchConfiguration('color_width').perform(context))
        color_height = int(LaunchConfiguration('color_height').perform(context))
        color_fps = int(LaunchConfiguration('color_fps').perform(context))
        vr_fps = int(LaunchConfiguration('vr_fps').perform(context))
        video_bitrate_kbps = int(LaunchConfiguration('video_bitrate_kbps').perform(context))

    depth_width = int(LaunchConfiguration('depth_width').perform(context))
    depth_height = int(LaunchConfiguration('depth_height').perform(context))
    depth_fps = int(LaunchConfiguration('depth_fps').perform(context))
    udp_port = int(LaunchConfiguration('udp_port').perform(context))
    udp_host = LaunchConfiguration('udp_host').perform(context)
    tcp_port = int(LaunchConfiguration('tcp_port').perform(context))
    tcp_host = LaunchConfiguration('tcp_host').perform(context)
    keyframe_interval = int(LaunchConfiguration('keyframe_interval').perform(context))
    video_codec = LaunchConfiguration('video_codec').perform(context)

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
        output='screen'
    )

    vr_forwarder_node = Node(
        package='openarmx_head_vision_h264',
        executable='vr_video_forwarder',
        name='vr_video_forwarder',
        parameters=[{
            'target_fps': vr_fps,
            'video_bitrate_kbps': video_bitrate_kbps,
            'keyframe_interval': keyframe_interval,
            'video_codec': video_codec,
            'udp_port': udp_port,
            'udp_host': udp_host,
            'tcp_port': tcp_port,
            'tcp_host': tcp_host,
            'max_queue_size': 1,
            'flip_vertical': ParameterValue(LaunchConfiguration('flip_vertical'), value_type=bool),
            'flip_horizontal': ParameterValue(LaunchConfiguration('flip_horizontal'), value_type=bool),
            'image_reliability': 'reliable',
            'enable_stats': True,
            'stats_interval': 5.0,
            'enable_depth': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
        }],
        remappings=[
            ('/image_input', '/vision/color/image_raw'),
        ],
        output='screen'
    )

    return [realsense_driver, source_node, vr_forwarder_node]
