#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_camera_serial(camera_key: str) -> str:
    config_path = Path.home() / '.openflex' / 'cameras_config.yaml'
    if not config_path.exists():
        return ''
    try:
        with config_path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        return str(data.get('cameras', {}).get(camera_key, {}).get('serial_no', '')).strip()
    except Exception:
        return ''


def _as_bool(context, argument_name: str) -> bool:
    return LaunchConfiguration(argument_name).perform(context).strip().lower() in {
        '1', 'true', 'yes', 'on'
    }


def _serial_param(context, argument_name: str) -> str:
    serial = LaunchConfiguration(argument_name).perform(context).strip()
    if not serial:
        return ''
    return serial if serial.startswith('_') else f'_{serial}'


def _create_camera(context, side: str, profile: str) -> Node:
    camera_name = f'cam_{side}'
    camera_type = LaunchConfiguration(f'{camera_name}_type').perform(context).strip().upper()
    if camera_type not in {'D405', 'D435', 'D435I'}:
        raise RuntimeError(f'Unsupported {camera_name}_type={camera_type}')
    serial = _serial_param(context, f'{camera_name}_serial')
    if not serial:
        raise RuntimeError(f'{camera_name}_serial is required')

    option_module = 'depth_module' if camera_type == 'D405' else 'rgb_camera'
    profile_parameter = {f'{option_module}.color_profile': profile}
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=camera_name,
        parameters=[{
            'serial_no': serial,
            'initial_reset': _as_bool(context, 'hand_camera_initial_reset'),
            'enable_color': True,
            'enable_depth': False,
            'align_depth.enable': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'pointcloud.enable': False,
            f'{option_module}.enable_auto_exposure': _as_bool(context, 'hand_enable_auto_exposure'),
            f'{option_module}.enable_auto_white_balance': _as_bool(context, 'hand_enable_auto_white_balance'),
            f'{option_module}.backlight_compensation': _as_bool(context, 'hand_backlight_compensation'),
            f'{option_module}.brightness': int(LaunchConfiguration('hand_brightness').perform(context)),
            **profile_parameter,
        }],
        remappings=[
            (f'/{camera_name}/{camera_name}/color/image_raw', f'/{camera_name}/color/image_raw'),
            (f'/{camera_name}/{camera_name}/color/camera_info', f'/{camera_name}/color/camera_info'),
        ],
        output='screen',
    )


def _create_source(side: str) -> Node:
    camera_name = f'cam_{side}'
    return Node(
        package='openarmx_head_vision_h264',
        executable='camera_stream_source',
        name=f'{side}_wrist_camera_stream_source',
        parameters=[{
            'enable_depth': False,
            'color_input_topic': f'/{camera_name}/color/image_raw',
            'color_camera_info_input_topic': f'/{camera_name}/color/camera_info',
            'color_output_topic': f'/vision/{side}/color/image_raw',
            'color_camera_info_output_topic': f'/vision/{side}/color/camera_info',
            'image_reliability': 'reliable',
            'enable_stats': True,
            'stats_interval': 5.0,
        }],
        output='screen',
    )


def _launch_setup(context):
    width = int(LaunchConfiguration('hand_color_width').perform(context))
    height = int(LaunchConfiguration('hand_color_height').perform(context))
    fps = int(LaunchConfiguration('hand_color_fps').perform(context))
    profile = f'{width}x{height}x{fps}'
    left = LaunchConfiguration('cam_left_serial').perform(context).strip()
    right = LaunchConfiguration('cam_right_serial').perform(context).strip()
    if left and right and left.lstrip('_') == right.lstrip('_'):
        raise RuntimeError('cam_left_serial and cam_right_serial must be different')

    actions = []
    delay = max(0.0, float(LaunchConfiguration('right_camera_start_delay_sec').perform(context)))
    for side, port_argument in (('left', 'left_udp_port'), ('right', 'right_udp_port')):
        camera_name = f'cam_{side}'
        stream_actions = [
            _create_camera(context, side, profile),
            _create_source(side),
            Node(
                package='openarmx_head_vision_h264',
                executable='vr_video_forwarder',
                name=f'{side}_wrist_vr_video_forwarder',
                parameters=[{
                    'target_fps': LaunchConfiguration('hand_vr_fps'),
                    'video_bitrate_kbps': LaunchConfiguration('hand_video_bitrate_kbps'),
                    'keyframe_interval': LaunchConfiguration('keyframe_interval'),
                    'video_codec': 'h264',
                    'image_topic': f'/vision/{side}/color/image_raw',
                    'udp_port': LaunchConfiguration(port_argument),
                    'udp_host': LaunchConfiguration('ros_ip'),
                    'max_queue_size': 1,
                    'flip_vertical': ParameterValue(LaunchConfiguration('flip_vertical'), value_type=bool),
                    'flip_horizontal': ParameterValue(LaunchConfiguration('flip_horizontal'), value_type=bool),
                    'image_reliability': 'reliable',
                    'enable_stats': True,
                    'stats_interval': 5.0,
                    'enable_depth': False,
                }],
                output='screen',
            ),
        ]
        if side == 'right' and delay > 0:
            actions.append(TimerAction(period=delay, actions=stream_actions))
        else:
            actions.extend(stream_actions)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('cam_left_serial', default_value=_default_camera_serial('left_wrist')),
        DeclareLaunchArgument('cam_right_serial', default_value=_default_camera_serial('right_wrist')),
        DeclareLaunchArgument('cam_left_type', default_value='D405'),
        DeclareLaunchArgument('cam_right_type', default_value='D405'),
        DeclareLaunchArgument('hand_camera_initial_reset', default_value='true'),
        DeclareLaunchArgument('right_camera_start_delay_sec', default_value='2.0'),
        DeclareLaunchArgument('hand_color_width', default_value='424'),
        DeclareLaunchArgument('hand_color_height', default_value='240'),
        DeclareLaunchArgument('hand_color_fps', default_value='30'),
        DeclareLaunchArgument('hand_vr_fps', default_value='15'),
        DeclareLaunchArgument('hand_video_bitrate_kbps', default_value='1000'),
        DeclareLaunchArgument('hand_enable_auto_exposure', default_value='true'),
        DeclareLaunchArgument('hand_enable_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('hand_backlight_compensation', default_value='true'),
        DeclareLaunchArgument('hand_brightness', default_value='32'),
        DeclareLaunchArgument('left_udp_port', default_value='5601'),
        DeclareLaunchArgument('right_udp_port', default_value='5602'),
        DeclareLaunchArgument('keyframe_interval', default_value='30'),
        DeclareLaunchArgument('flip_vertical', default_value='true'),
        DeclareLaunchArgument('flip_horizontal', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
