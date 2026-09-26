#!/usr/bin/env python3
"""
VR Forwarder Only Launch File

Only launches the VR video forwarder node.
Useful when you want to forward images from any existing image topic.

Usage:
    # Forward from default topic
    ros2 launch openarmx_head_vision_h264 vr_forwarder_only.launch.py

    # Forward from custom topic
    ros2 launch openarmx_head_vision_h264 vr_forwarder_only.launch.py \\
        image_topic:=/my_camera/image_raw

    # Custom VR parameters
    ros2 launch openarmx_head_vision_h264 vr_forwarder_only.launch.py \\
        vr_fps:=25 video_bitrate_kbps:=6000 keyframe_interval:=30 udp_port:=5601
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for VR forwarder only."""

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'image_topic',
            default_value='/vision/color/image_raw',
            description='Input image topic to forward to VR'
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
            'flip_vertical',
            default_value='true',
            description='Flip the host-side image vertically before video encoding'
        ),

        DeclareLaunchArgument(
            'flip_horizontal',
            default_value='false',
            description='Flip the host-side image horizontally before video encoding'
        ),

        # VR video forwarder node
        Node(
            package='openarmx_head_vision_h264',
            executable='vr_video_forwarder',
            name='vr_video_forwarder',
            parameters=[{
                'target_fps': LaunchConfiguration('vr_fps'),
                'video_bitrate_kbps': LaunchConfiguration('video_bitrate_kbps'),
                'keyframe_interval': LaunchConfiguration('keyframe_interval'),
                'video_codec': LaunchConfiguration('video_codec'),
                'image_topic': LaunchConfiguration('image_topic'),
                'udp_port': LaunchConfiguration('udp_port'),
                'udp_host': LaunchConfiguration('udp_host'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'tcp_host': '0.0.0.0',
                'max_queue_size': 1,
                'flip_vertical': LaunchConfiguration('flip_vertical'),
                'flip_horizontal': LaunchConfiguration('flip_horizontal'),
                'enable_stats': True,
                'stats_interval': 5.0,
                'enable_depth': False,
            }],
            output='screen'
        ),
    ])
