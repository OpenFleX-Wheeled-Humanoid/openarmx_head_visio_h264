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


def _default_camera_serial(camera_key: str) -> str:
    config_path = Path.home() / ".openflex" / "cameras_config.yaml"
    if not config_path.exists():
        return ""
    try:
        with config_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return str(data.get("cameras", {}).get(camera_key, {}).get("serial_no", "")).strip()
    except Exception:
        return ""


def _default_head_serial() -> str:
    return _default_camera_serial('head')


def _serial_param(context, argument_name: str = 'serial_number') -> str:
    serial_number = LaunchConfiguration(argument_name).perform(context).strip()
    if not serial_number:
        return ''
    return serial_number if serial_number.startswith('_') else f'_{serial_number}'


def _as_bool(context, argument_name: str) -> bool:
    return LaunchConfiguration(argument_name).perform(context).strip().lower() in {
        '1', 'true', 'yes', 'on'
    }


def _create_wrist_camera(context, side: str, profile: str) -> Node:
    camera_name = f'cam_{side}'
    camera_type = LaunchConfiguration(f'{camera_name}_type').perform(context).strip().upper()
    if camera_type not in {'D405', 'D435', 'D435I'}:
        raise RuntimeError(
            f'Unsupported {camera_name}_type={camera_type}; use D405, D435, or D435I'
        )

    serial_number = _serial_param(context, f'{camera_name}_serial')
    if not serial_number:
        raise RuntimeError(f'{camera_name}_serial is required when enable_hand_video=true')

    if camera_type == 'D405':
        profile_parameters = {'depth_module.color_profile': profile}
        option_module = 'depth_module'
    else:
        profile_parameters = {'rgb_camera.color_profile': profile}
        option_module = 'rgb_camera'

    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=camera_name,
        parameters=[{
            'serial_no': serial_number,
            'enable_color': True,
            'enable_depth': False,
            'align_depth.enable': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'pointcloud.enable': False,
            f'{option_module}.enable_auto_exposure': _as_bool(
                context, 'hand_enable_auto_exposure'),
            f'{option_module}.enable_auto_white_balance': _as_bool(
                context, 'hand_enable_auto_white_balance'),
            f'{option_module}.backlight_compensation': _as_bool(
                context, 'hand_backlight_compensation'),
            f'{option_module}.brightness': int(
                LaunchConfiguration('hand_brightness').perform(context)),
            **profile_parameters,
        }],
        remappings=[
            (f'/{camera_name}/{camera_name}/color/image_raw',
             f'/{camera_name}/color/image_raw'),
            (f'/{camera_name}/{camera_name}/color/camera_info',
             f'/{camera_name}/color/camera_info'),
        ],
        output='screen',
    )


def _create_wrist_stream_source(side: str) -> Node:
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_mode',
            default_value='balanced',
            description='Configuration preset: balanced, high_quality, low_latency, bandwidth_saving, custom'
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value=_default_head_serial(),
            description='Head RealSense D435i serial number; defaults to ~/.openflex/cameras_config.yaml cameras.head.serial_no; empty means auto-select'
        ),
        DeclareLaunchArgument(
            'enable_head_video', default_value='true',
            description='Start and forward the head RealSense video on port 5600'
        ),
        DeclareLaunchArgument(
            'enable_hand_video', default_value='false',
            description='Legacy alias: start and forward both wrist videos'
        ),
        DeclareLaunchArgument(
            'enable_left_hand_video', default_value='false',
            description='Start and forward the left wrist video on port 5601'
        ),
        DeclareLaunchArgument(
            'enable_right_hand_video', default_value='false',
            description='Start and forward the right wrist video on port 5602'
        ),
        DeclareLaunchArgument(
            'cam_left_serial', default_value=_default_camera_serial('left_wrist'),
            description='Left wrist RealSense serial number'
        ),
        DeclareLaunchArgument(
            'cam_right_serial', default_value=_default_camera_serial('right_wrist'),
            description='Right wrist RealSense serial number'
        ),
        DeclareLaunchArgument('cam_left_type', default_value='D405'),
        DeclareLaunchArgument('cam_right_type', default_value='D405'),
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
            'stream_layout': 'mono',
            'udp_port': udp_port,
            'udp_host': udp_host,
            'tcp_port': tcp_port,
            'tcp_host': tcp_host,
            'max_queue_size': 1,
            'flip_vertical': ParameterValue(LaunchConfiguration('flip_vertical'), value_type=bool),
            'flip_horizontal': ParameterValue(LaunchConfiguration('flip_horizontal'), value_type=bool),
            'align_output_to_macroblocks': True,
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

    enable_head_video = _as_bool(context, 'enable_head_video')
    legacy_hand_video = _as_bool(context, 'enable_hand_video')
    enable_left_hand_video = legacy_hand_video or _as_bool(context, 'enable_left_hand_video')
    enable_right_hand_video = legacy_hand_video or _as_bool(context, 'enable_right_hand_video')
    if not enable_head_video and not enable_left_hand_video and not enable_right_hand_video:
        raise RuntimeError(
            'At least one of enable_head_video, enable_left_hand_video, or '
            'enable_right_hand_video must be true'
        )

    actions = []
    if enable_head_video:
        actions.extend([realsense_driver, source_node, vr_forwarder_node])

    if enable_left_hand_video or enable_right_hand_video:
        left_serial = LaunchConfiguration('cam_left_serial').perform(context).strip()
        right_serial = LaunchConfiguration('cam_right_serial').perform(context).strip()
        if (
            enable_left_hand_video and enable_right_hand_video and left_serial and right_serial
            and left_serial.lstrip('_') == right_serial.lstrip('_')
        ):
            raise RuntimeError('cam_left_serial and cam_right_serial must be different')

        hand_width = int(LaunchConfiguration('hand_color_width').perform(context))
        hand_height = int(LaunchConfiguration('hand_color_height').perform(context))
        hand_fps = int(LaunchConfiguration('hand_color_fps').perform(context))
        hand_profile = f'{hand_width}x{hand_height}x{hand_fps}'
        hand_vr_fps = int(LaunchConfiguration('hand_vr_fps').perform(context))
        hand_bitrate = int(LaunchConfiguration('hand_video_bitrate_kbps').perform(context))

        selected_sides = []
        if enable_left_hand_video:
            selected_sides.append(('left', 'left_udp_port'))
        if enable_right_hand_video:
            selected_sides.append(('right', 'right_udp_port'))
        for side, port_argument in selected_sides:
            port = int(LaunchConfiguration(port_argument).perform(context))
            actions.extend([
                _create_wrist_camera(context, side, hand_profile),
                _create_wrist_stream_source(side),
                Node(
                    package='openarmx_head_vision_h264',
                    executable='vr_video_forwarder',
                    name=f'{side}_wrist_vr_video_forwarder',
                    parameters=[{
                        'target_fps': hand_vr_fps,
                        'video_bitrate_kbps': hand_bitrate,
                        'keyframe_interval': keyframe_interval,
                        'video_codec': video_codec,
                        'stream_layout': 'mono',
                        'image_topic': f'/vision/{side}/color/image_raw',
                        'udp_port': port,
                        'udp_host': udp_host,
                        'tcp_port': port,
                        'tcp_host': tcp_host,
                        'max_queue_size': 1,
                        'flip_vertical': ParameterValue(
                            LaunchConfiguration('flip_vertical'), value_type=bool),
                        'flip_horizontal': ParameterValue(
                            LaunchConfiguration('flip_horizontal'), value_type=bool),
                        'align_output_to_macroblocks': True,
                        'image_reliability': 'reliable',
                        'enable_stats': True,
                        'stats_interval': 5.0,
                        'enable_depth': False,
                    }],
                    output='screen',
                ),
            ])

    return actions
