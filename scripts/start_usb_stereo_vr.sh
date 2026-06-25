#!/bin/bash

# USB 双目相机 VR 视频流启动脚本

echo "正在启动 USB 双目相机 VR 视频流..."

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# 加载 ROS 环境
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 参数
VIDEO_DEVICE=${1:-auto}
ROS_IP=${2:-0.0.0.0}
LEFT_PORT=${3:-5600}
RIGHT_PORT=${4:-5601}
BITRATE=${5:-1500}
FRAMERATE=${6:-15}
IMAGE_WIDTH=${7:-1280}
IMAGE_HEIGHT=${8:-480}
CAMERA_FPS=${9:-30}

echo "参数配置:"
echo "  视频设备: $VIDEO_DEVICE"
echo "  UDP IP: $ROS_IP"
echo "  左眼端口: $LEFT_PORT"
echo "  右眼端口: $RIGHT_PORT"
echo "  码率: $BITRATE kbps"
echo "  目标帧率: $FRAMERATE fps"
echo "  相机采集帧率: $CAMERA_FPS fps"
echo "  拼接图分辨率: ${IMAGE_WIDTH}x${IMAGE_HEIGHT}"
echo ""

if [ "$VIDEO_DEVICE" = "auto" ]; then
  echo "自动识别支持 ${IMAGE_WIDTH}x${IMAGE_HEIGHT} 的双目相机..."
  VIDEO_DEVICE=""
  for DEV in /dev/video*; do
    if v4l2-ctl -d "$DEV" --list-formats-ext 2>/dev/null | grep -Eq "${IMAGE_WIDTH}[[:space:]]*x[[:space:]]*${IMAGE_HEIGHT}"; then
      VIDEO_DEVICE="$DEV"
      break
    fi
  done
  if [ -z "$VIDEO_DEVICE" ]; then
    echo "错误: 未找到支持 ${IMAGE_WIDTH}x${IMAGE_HEIGHT} 的双目相机"
    exit 1
  fi
  echo "已选择双目相机: $VIDEO_DEVICE"
fi

# 启动 USB 相机
echo "启动 USB 相机 (${IMAGE_WIDTH}x${IMAGE_HEIGHT} @ ${CAMERA_FPS}fps)..."
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __node:=usb_stereo_camera \
  -p video_device:="$VIDEO_DEVICE" \
  -p image_width:=$IMAGE_WIDTH \
  -p image_height:=$IMAGE_HEIGHT \
  -p framerate:=$CAMERA_FPS \
  -p pixel_format:=mjpeg2rgb \
  -p camera_name:=stereo_camera \
  -p io_method:=mmap &

USB_CAM_PID=$!
sleep 2

# 启动图像分割器
echo "启动立体图像分割器..."
ros2 run openarmx_head_vision_h264 stereo_splitter --ros-args \
  -p input_topic:=/image_raw \
  -p left_output_topic:=/vision/stereo/left/image_raw \
  -p right_output_topic:=/vision/stereo/right/image_raw &

SPLITTER_PID=$!
sleep 1

# 启动左眼视频转发器
echo "启动左眼视频转发器 (端口 $LEFT_PORT)..."
ros2 run openarmx_head_vision_h264 vr_video_forwarder --ros-args \
  -r __node:=vr_forwarder_left \
  -p image_topic:=/vision/stereo/left/image_raw \
  -p udp_host:="$ROS_IP" \
  -p udp_port:=$LEFT_PORT \
  -p video_codec:=h264 \
  -p video_bitrate_kbps:=$BITRATE \
  -p target_fps:=$FRAMERATE \
  -p enable_stats:=true \
  -p stats_interval:=5.0 &

LEFT_FWD_PID=$!

# 启动右眼视频转发器
echo "启动右眼视频转发器 (端口 $RIGHT_PORT)..."
ros2 run openarmx_head_vision_h264 vr_video_forwarder --ros-args \
  -r __node:=vr_forwarder_right \
  -p image_topic:=/vision/stereo/right/image_raw \
  -p udp_host:="$ROS_IP" \
  -p udp_port:=$RIGHT_PORT \
  -p video_codec:=h264 \
  -p video_bitrate_kbps:=$BITRATE \
  -p target_fps:=$FRAMERATE \
  -p enable_stats:=true \
  -p stats_interval:=5.0 &

RIGHT_FWD_PID=$!

echo ""
echo "✓ 所有节点已启动"
echo "  USB 相机 PID: $USB_CAM_PID"
echo "  图像分割器 PID: $SPLITTER_PID"
echo "  左眼转发器 PID: $LEFT_FWD_PID"
echo "  右眼转发器 PID: $RIGHT_FWD_PID"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 捕获退出信号
trap "echo '正在停止...'; kill $USB_CAM_PID $SPLITTER_PID $LEFT_FWD_PID $RIGHT_FWD_PID 2>/dev/null; exit 0" SIGINT SIGTERM

# 等待
wait
