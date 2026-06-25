#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import re
import subprocess


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

    pattern = re.compile(rf'\b{width}\s*x\s*{height}\b')
    return pattern.search(result.stdout) is not None


def _resolve_video_device(requested_device: str, width: int, height: int) -> str:
    if requested_device and requested_device != 'auto':
        if not Path(requested_device).exists():
            raise RuntimeError(f'Configured video_device does not exist: {requested_device}')
        if not _device_supports_format(requested_device, width, height):
            raise RuntimeError(
                f'{requested_device} does not support {width}x{height}. '
                'Use video_device:=auto or choose the stereo USB camera device.'
            )
        return requested_device

    candidates = sorted(Path('/dev').glob('video*'), key=lambda p: str(p))
    matches = [
        str(device)
        for device in candidates
        if _device_supports_format(str(device), width, height)
    ]

    if not matches:
        raise RuntimeError(
            f'No /dev/video* device supports stereo frame size {width}x{height}. '
            'Check the USB stereo camera connection or set image_width/image_height to a supported side-by-side size.'
        )

    return matches[0]


def launch_setup(context):
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    requested_device = LaunchConfiguration('video_device').perform(context).strip()
    resolved_device = _resolve_video_device(requested_device, image_width, image_height)
    print(f'[usb_stereo_vr] selected stereo camera: {resolved_device} ({image_width}x{image_height})')

    return [
        # USB 双目相机节点。默认 1280x480 是左右拼接图，拆分后每眼 640x480。
        Node(
            package='openarmx_head_vision_h264',
            executable='v4l2_mjpeg_camera',
            name='usb_stereo_camera',
            parameters=[{
                'video_device': resolved_device,
                'output_topic': '/image_raw',
                'compressed_output_topic': '/image_raw/compressed',
                'frame_id': 'stereo_camera',
                'image_width': image_width,
                'image_height': image_height,
                'camera_fps': LaunchConfiguration('camera_fps'),
                'publish_raw': True,
                'publish_compressed': True,
                'enable_stats': True,
                'stats_interval': 5.0,
            }],
            output='screen'
        ),

        # 双目图像分割节点（将左右拼接图分割为左右两路）
        Node(
            package='openarmx_head_vision_h264',
            executable='stereo_splitter',
            name='stereo_splitter',
            parameters=[{
                'input_topic': '/image_raw',
                'left_output_topic': '/vision/stereo/left/image_raw',
                'right_output_topic': '/vision/stereo/right/image_raw',
                'enable_stats': True,
                'stats_interval': 5.0,
            }],
            output='screen'
        ),

        # 左眼视频转发器
        Node(
            package='openarmx_head_vision_h264',
            executable='vr_video_forwarder',
            name='vr_forwarder_left',
            parameters=[{
                'image_topic': '/vision/stereo/left/image_raw',
                'udp_host': LaunchConfiguration('ros_ip'),
                'udp_port': LaunchConfiguration('left_port'),
                'video_codec': 'h264',
                'video_bitrate_kbps': LaunchConfiguration('bitrate'),
                'target_fps': LaunchConfiguration('framerate'),
                'enable_stats': True,
                'stats_interval': 5.0,
            }],
            output='screen'
        ),

        # 右眼视频转发器
        Node(
            package='openarmx_head_vision_h264',
            executable='vr_video_forwarder',
            name='vr_forwarder_right',
            parameters=[{
                'image_topic': '/vision/stereo/right/image_raw',
                'udp_host': LaunchConfiguration('ros_ip'),
                'udp_port': LaunchConfiguration('right_port'),
                'video_codec': 'h264',
                'video_bitrate_kbps': LaunchConfiguration('bitrate'),
                'target_fps': LaunchConfiguration('framerate'),
                'enable_stats': True,
                'stats_interval': 5.0,
            }],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('left_port', default_value='5600'),
        DeclareLaunchArgument('right_port', default_value='5601'),
        DeclareLaunchArgument('bitrate', default_value='1500'),
        DeclareLaunchArgument('framerate', default_value='15'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('video_device', default_value='auto'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='480'),
        OpaqueFunction(function=launch_setup),
    ])
