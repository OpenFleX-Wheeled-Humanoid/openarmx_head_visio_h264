#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import re
import subprocess
import os


def _device_supports_format(device_path: str, width: int, height: int) -> bool:
    try:
        result = subprocess.run(
            ['v4l2-ctl', '-d', device_path, '--list-formats-ext'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return False
    if result.returncode != 0:
        return False
    return re.search(rf'\b{width}\s*x\s*{height}\b', result.stdout) is not None


def _resolve_video_device(requested_device: str, width: int, height: int) -> str:
    if requested_device and requested_device != 'auto':
        if not Path(requested_device).exists():
            raise RuntimeError(f'Configured video_device does not exist: {requested_device}')
        if not _device_supports_format(requested_device, width, height):
            raise RuntimeError(f'{requested_device} does not support {width}x{height}')
        return requested_device

    for device in sorted(Path('/dev').glob('video*'), key=lambda p: str(p)):
        if _device_supports_format(str(device), width, height):
            return str(device)

    raise RuntimeError(f'No /dev/video* device supports side-by-side stereo size {width}x{height}')


def launch_setup(context):
    pkg_share = get_package_share_directory('openarmx_head_vision_h264')
    profile_name = LaunchConfiguration('camera_profile').perform(context).strip()
    profile_file = None
    if profile_name and profile_name != 'none':
        profile_path = Path(profile_name)
        if not profile_path.is_absolute():
            profile_path = Path(pkg_share) / 'config' / profile_name
        if not profile_path.exists():
            raise RuntimeError(f'camera_profile file does not exist: {profile_path}')
        profile_file = str(profile_path)

    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    requested_device = LaunchConfiguration('video_device').perform(context).strip()
    resolved_device = _resolve_video_device(requested_device, image_width, image_height)
    print(f'[usb_stereo_side_by_side_vr] selected stereo camera: {resolved_device} ({image_width}x{image_height})')

    return [
        Node(
            package='openarmx_head_vision_h264',
            executable='v4l2_mjpeg_camera',
            name='usb_stereo_camera',
            parameters=(([profile_file] if profile_file else []) + [{
                'video_device': resolved_device,
                'output_topic': '/image_raw',
                'compressed_output_topic': '/image_raw/compressed',
                'frame_id': 'stereo_camera',
                'image_width': image_width,
                'image_height': image_height,
                'camera_fps': LaunchConfiguration('camera_fps'),
                'publish_raw': ParameterValue(LaunchConfiguration('publish_raw'), value_type=bool),
                'publish_compressed': True,
                'enable_stats': True,
                'stats_interval': 5.0,
            }]),
            output='screen'
        ),
        Node(
            package='openarmx_head_vision_h264',
            executable='vr_video_forwarder',
            name='vr_stereo_side_by_side_forwarder',
            parameters=(([profile_file] if profile_file else []) + [{
                'image_topic': '/image_raw',
                'compressed_image_topic': '/image_raw/compressed',
                'prefer_compressed_image': True,
                'udp_host': LaunchConfiguration('ros_ip'),
                'udp_port': LaunchConfiguration('udp_port'),
                'video_codec': 'h264',
                'video_bitrate_kbps': LaunchConfiguration('bitrate'),
                'target_fps': LaunchConfiguration('framerate'),
                'enable_stats': True,
                'stats_interval': 5.0,
            }]),
            output='screen'
        ),
        Node(
            package='openarmx_head_vision_h264',
            executable='v4l2_auto_exposure_controller',
            name='usb_stereo_auto_exposure_controller',
            parameters=[{
                'enabled': ParameterValue(LaunchConfiguration('auto_exposure_control'), value_type=bool),
                'video_device': resolved_device,
                'compressed_image_topic': '/image_raw/compressed',
                'prefer_compressed_image': True,
                'target_luma': ParameterValue(LaunchConfiguration('auto_target_luma'), value_type=float),
                'deadband_luma': ParameterValue(LaunchConfiguration('auto_deadband_luma'), value_type=float),
                'min_exposure': ParameterValue(LaunchConfiguration('auto_min_exposure'), value_type=int),
                'max_exposure': ParameterValue(LaunchConfiguration('auto_max_exposure'), value_type=int),
                'min_gain': ParameterValue(LaunchConfiguration('auto_min_gain'), value_type=int),
                'max_gain': ParameterValue(LaunchConfiguration('auto_max_gain'), value_type=int),
                'exposure_step': ParameterValue(LaunchConfiguration('auto_exposure_step'), value_type=int),
                'gain_step': ParameterValue(LaunchConfiguration('auto_gain_step'), value_type=int),
                'update_interval': ParameterValue(LaunchConfiguration('auto_update_interval'), value_type=float),
                'startup_auto_seconds': ParameterValue(LaunchConfiguration('auto_startup_seconds'), value_type=float),
                'set_sharpness': ParameterValue(LaunchConfiguration('auto_sharpness'), value_type=int),
                'power_line_frequency': ParameterValue(LaunchConfiguration('auto_power_line_frequency'), value_type=int),
            }],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_port', default_value='5600'),
        DeclareLaunchArgument('bitrate', default_value='8000'),
        DeclareLaunchArgument('framerate', default_value='30'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('video_device', default_value='auto'),
        DeclareLaunchArgument('image_width', default_value='2560'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('publish_raw', default_value='false'),
        DeclareLaunchArgument('camera_profile', default_value='usb_stereo_flipped_camera.yaml'),
        DeclareLaunchArgument('auto_exposure_control', default_value='true'),
        DeclareLaunchArgument('auto_target_luma', default_value='80.0'),
        DeclareLaunchArgument('auto_deadband_luma', default_value='8.0'),
        DeclareLaunchArgument('auto_min_exposure', default_value='60'),
        DeclareLaunchArgument('auto_max_exposure', default_value='720'),
        DeclareLaunchArgument('auto_min_gain', default_value='0'),
        DeclareLaunchArgument('auto_max_gain', default_value='60'),
        DeclareLaunchArgument('auto_exposure_step', default_value='12'),
        DeclareLaunchArgument('auto_gain_step', default_value='8'),
        DeclareLaunchArgument('auto_update_interval', default_value='0.5'),
        DeclareLaunchArgument('auto_startup_seconds', default_value='1.5'),
        DeclareLaunchArgument('auto_sharpness', default_value='2'),
        DeclareLaunchArgument('auto_power_line_frequency', default_value='1'),
        OpaqueFunction(function=launch_setup),
    ])
