# OpenArmX Head Vision User Guide

English | [中文](./README-CN.md)

---

![Cover](./image/cover.gif)


This package is used to send camera images to the VR side. Currently, there are two camera links:

- RealSense D435/D435i: Used only as a monocular color camera.
- Independent USB stereo camera: Uses left-right stitched images, ROS side splits them into left eye and right eye streams.

RealSense is not used as a stereo camera.

## Preparation

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
```

If you modified the code, build first:

```bash
colcon build --packages-select openarmx_head_visio_h264
source install/setup.bash
```

## RealSense Monocular

Launch RealSense color camera by default and send to VR:

```bash
ros2 launch openarmx_head_visio_h264 d435i_vr.launch.py
```

Specify RealSense serial number:

Check camera ID:

```bash
rs-enumerate-devices -s
```

The `Serial Number` in the output is the camera ID, for example:

```text
Device Name                   Serial Number
Intel RealSense D435          150622072782
```

```bash
ros2 launch openarmx_head_visio_h264 d435i_vr.launch.py serial_number:=150622072782
```

Custom resolution:

```bash
ros2 launch openarmx_head_visio_h264 d435i_vr.launch.py \
  config_mode:=custom \
  color_width:=1280 \
  color_height:=720 \
  color_fps:=30
```

VR side parameters:

- `enable_stereo`: `false`
- Port: `5600`

## USB Stereo Camera

The independent stereo camera input is a left-right stitched image, such as `3840x1080`, ROS side will split it into:

- Left eye: `/vision/stereo/left/image_raw`
- Right eye: `/vision/stereo/right/image_raw`

Launch:

```bash
ros2 launch openarmx_head_visio_h264 usb_stereo_side_by_side_vr.launch.py
```

Specify device and port:

```bash
ros2 launch openarmx_head_visio_h264 usb_stereo_side_by_side_vr.launch.py \
  ros_ip:=0.0.0.0 \
  video_device:=auto \
  udp_port:=5600 \
  bitrate:=1500 \
  framerate:=15 \
  camera_fps:=30
```

`video_device:=auto` will automatically select a stereo camera that supports the current stitched resolution, such as the default `1280x480`.
`camera_fps` is the camera capture frame rate, default `30`; `framerate` is the target frame rate sent to VR, default `15`.

VR side parameters:

- `enable_stereo`: `true`
- `left_port`: `5600`
- `right_port`: `5601`

## Forward Existing Image Topics Only

If you already have ROS image topics, you can only start the video forwarder:

```bash
ros2 launch openarmx_head_visio_h264 vr_forwarder_only.launch.py \
  image_topic:=/vision/color/image_raw \
  udp_port:=5600
```

## Check Host IP

```bash
hostname -I
```

Enter the IP of the computer running ROS on the VR side.

## Normal Logs

After receiving a VR connection, you will see:

```text
VR UDP client registered from <vr_ip>:<port>
```

When sending is normal, statistics will grow:

```text
Stats: Recv=... Enc=... Sent=... Pkts=... Hello=... Client=...
```

If `Recv` grows but `Enc/Sent` does not, it usually means VR is not connected or the port is wrong.

## License

This work is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE_CN.md](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robot platform ecosystem, developed specifically for research and industrial applications in the collaborative robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | <https://openarmx.com/> |
| 🌐 Docs | <http://docs.openarmx.com/> |
| 📍 Address | Tianjin Economic-Technological Development Area West Zone, No. 11 Xinyeba Street, Huacheng Machinery Factory |
| 👤 Contact Person | Mr. Wang |
