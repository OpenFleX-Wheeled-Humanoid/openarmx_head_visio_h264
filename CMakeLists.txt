cmake_minimum_required(VERSION 3.22)
project(openarmx_head_vision_h264)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

add_executable(camera_stream_source
  src/camera_stream_source.cpp
)

target_compile_features(camera_stream_source PUBLIC cxx_std_17)

ament_target_dependencies(camera_stream_source
  rclcpp
  sensor_msgs
)

add_executable(vr_video_forwarder
  src/vr_video_forwarder.cpp
)

target_compile_features(vr_video_forwarder PUBLIC cxx_std_17)

target_link_libraries(vr_video_forwarder
  ${OpenCV_LIBRARIES}
  PkgConfig::FFMPEG
)

ament_target_dependencies(vr_video_forwarder
  rclcpp
  sensor_msgs
  cv_bridge
)

add_executable(v4l2_mjpeg_camera
  src/v4l2_mjpeg_camera.cpp
)

target_compile_features(v4l2_mjpeg_camera PUBLIC cxx_std_17)

target_link_libraries(v4l2_mjpeg_camera
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(v4l2_mjpeg_camera
  rclcpp
  sensor_msgs
)

add_executable(v4l2_auto_exposure_controller
  src/v4l2_auto_exposure_controller.cpp
)

target_compile_features(v4l2_auto_exposure_controller PUBLIC cxx_std_17)

target_link_libraries(v4l2_auto_exposure_controller
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(v4l2_auto_exposure_controller
  rclcpp
  sensor_msgs
)

add_executable(stereo_splitter
  src/stereo_splitter.cpp
)

target_compile_features(stereo_splitter PUBLIC cxx_std_17)

target_link_libraries(stereo_splitter
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(stereo_splitter
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS camera_stream_source vr_video_forwarder v4l2_mjpeg_camera v4l2_auto_exposure_controller stereo_splitter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml README.md README-CN.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
