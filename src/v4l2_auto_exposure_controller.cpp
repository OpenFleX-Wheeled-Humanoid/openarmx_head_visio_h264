#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

int xioctl(int fd, unsigned long request, void* arg) {
  int result = 0;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

int get_control(int fd, std::uint32_t id, int fallback) {
  v4l2_control control{};
  control.id = id;
  if (xioctl(fd, VIDIOC_G_CTRL, &control) < 0) {
    return fallback;
  }
  return control.value;
}

bool set_control(int fd, std::uint32_t id, int value) {
  v4l2_control control{};
  control.id = id;
  control.value = value;
  return xioctl(fd, VIDIOC_S_CTRL, &control) == 0;
}

}  // namespace

class V4L2AutoExposureController : public rclcpp::Node {
public:
  V4L2AutoExposureController() : Node("v4l2_auto_exposure_controller") {
    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    compressed_image_topic_ =
        declare_parameter<std::string>("compressed_image_topic", "/image_raw/compressed");
    prefer_compressed_image_ = declare_parameter<bool>("prefer_compressed_image", true);
    enabled_ = declare_parameter<bool>("enabled", false);
    target_luma_ = declare_parameter<double>("target_luma", 90.0);
    deadband_luma_ = declare_parameter<double>("deadband_luma", 8.0);
    min_exposure_ = declare_parameter<int>("min_exposure", 60);
    max_exposure_ = declare_parameter<int>("max_exposure", 360);
    min_gain_ = declare_parameter<int>("min_gain", 0);
    max_gain_ = declare_parameter<int>("max_gain", 96);
    exposure_step_ = declare_parameter<int>("exposure_step", 12);
    gain_step_ = declare_parameter<int>("gain_step", 8);
    update_interval_ = declare_parameter<double>("update_interval", 0.5);
    startup_auto_seconds_ = declare_parameter<double>("startup_auto_seconds", 1.5);
    set_sharpness_ = declare_parameter<int>("set_sharpness", 1);
    power_line_frequency_ = declare_parameter<int>("power_line_frequency", 1);

    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "V4L2 auto exposure controller disabled");
      return;
    }

    fd_ = ::open(video_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + video_device_ + ": " + std::strerror(errno));
    }

    set_control(fd_, V4L2_CID_SHARPNESS, set_sharpness_);
    set_control(fd_, V4L2_CID_POWER_LINE_FREQUENCY, power_line_frequency_);
    set_control(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY);
    startup_until_ = now() + rclcpp::Duration::from_seconds(startup_auto_seconds_);

    if (prefer_compressed_image_) {
      compressed_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          compressed_image_topic_, rclcpp::QoS(1).best_effort().keep_last(1),
          std::bind(&V4L2AutoExposureController::compressed_callback, this, std::placeholders::_1));
    } else {
      image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          image_topic_, rclcpp::QoS(1).best_effort().keep_last(1),
          std::bind(&V4L2AutoExposureController::image_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(),
                "V4L2 auto exposure controller started: device=%s target_luma=%.1f "
                "exposure=[%d,%d] gain=[%d,%d]",
                video_device_.c_str(), target_luma_, min_exposure_, max_exposure_,
                min_gain_, max_gain_);
  }

  ~V4L2AutoExposureController() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!enabled_ || msg->data.empty()) {
      return;
    }
    cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                    const_cast<std::uint8_t*>(msg->data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (!image.empty()) {
      process_frame(image);
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!enabled_ || msg->data.empty() || msg->height == 0 || msg->width == 0) {
      return;
    }
    const int channels = static_cast<int>(msg->step / msg->width);
    if (channels < 1) {
      return;
    }
    cv::Mat image(static_cast<int>(msg->height), static_cast<int>(msg->width),
                  channels >= 3 ? CV_8UC3 : CV_8UC1,
                  const_cast<std::uint8_t*>(msg->data.data()), msg->step);
    process_frame(image);
  }

  void process_frame(const cv::Mat& image) {
    const auto current_time = now();
    if ((current_time - last_update_).seconds() < update_interval_) {
      return;
    }
    last_update_ = current_time;

    cv::Mat gray;
    if (image.channels() == 1) {
      gray = image;
    } else {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    const double luma = robust_luma(gray);
    if (current_time < startup_until_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "startup auto exposure: luma=%.1f", luma);
      return;
    }

    if (!manual_mode_) {
      exposure_ = std::clamp(get_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, 166),
                             min_exposure_, max_exposure_);
      gain_ = std::clamp(get_control(fd_, V4L2_CID_GAIN, 32), min_gain_, max_gain_);
      set_control(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
      set_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_);
      set_control(fd_, V4L2_CID_GAIN, gain_);
      manual_mode_ = true;
      RCLCPP_INFO(get_logger(), "switched to managed manual exposure: exposure=%d gain=%d",
                  exposure_, gain_);
    } else {
      const int device_exposure = get_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_);
      const int device_gain = get_control(fd_, V4L2_CID_GAIN, gain_);
      if (device_exposure < min_exposure_ || device_exposure > max_exposure_) {
        set_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_);
      }
      if (device_gain < min_gain_ || device_gain > max_gain_) {
        set_control(fd_, V4L2_CID_GAIN, gain_);
      }
    }

    int next_exposure = exposure_;
    int next_gain = gain_;
    const double error = target_luma_ - luma;

    if (std::abs(error) > deadband_luma_) {
      if (error > 0.0) {
        if (next_exposure < max_exposure_) {
          next_exposure = std::min(max_exposure_, next_exposure + exposure_step_);
        } else {
          next_gain = std::min(max_gain_, next_gain + gain_step_);
        }
      } else {
        if (next_gain > min_gain_) {
          next_gain = std::max(min_gain_, next_gain - gain_step_);
        }
        if (next_gain <= min_gain_ && next_exposure > min_exposure_) {
          next_exposure = std::max(min_exposure_, next_exposure - exposure_step_);
        }
      }
    }

    if (next_exposure != exposure_) {
      if (set_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, next_exposure)) {
        exposure_ = next_exposure;
      }
    }
    if (next_gain != gain_) {
      if (set_control(fd_, V4L2_CID_GAIN, next_gain)) {
        gain_ = next_gain;
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "managed exposure: luma=%.1f target=%.1f exposure=%d gain=%d "
                         "device_exposure=%d device_gain=%d",
                         luma, target_luma_, exposure_, gain_,
                         get_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_),
                         get_control(fd_, V4L2_CID_GAIN, gain_));
  }

  double robust_luma(const cv::Mat& gray) const {
    const int width = gray.cols;
    const int height = gray.rows;
    if (width <= 0 || height <= 0) {
      return 0.0;
    }

    std::vector<std::uint8_t> samples;
    samples.reserve(static_cast<std::size_t>((width / 8 + 1) * (height / 8 + 1)));
    for (int y = 0; y < height; y += 8) {
      const auto* row = gray.ptr<std::uint8_t>(y);
      for (int x = 0; x < width; x += 8) {
        samples.push_back(row[x]);
      }
    }
    if (samples.empty()) {
      return 0.0;
    }

    const auto lower_it = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() * 20 / 100);
    const auto upper_it = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() * 80 / 100);
    std::nth_element(samples.begin(), lower_it, samples.end());
    const auto lower = *lower_it;
    std::nth_element(samples.begin(), upper_it, samples.end());
    const auto upper = *upper_it;

    double sum = 0.0;
    int count = 0;
    for (const auto value : samples) {
      if (value >= lower && value <= upper) {
        sum += value;
        count++;
      }
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  }

  std::string video_device_;
  std::string image_topic_;
  std::string compressed_image_topic_;
  bool prefer_compressed_image_ = true;
  bool enabled_ = false;
  double target_luma_ = 90.0;
  double deadband_luma_ = 8.0;
  int min_exposure_ = 60;
  int max_exposure_ = 360;
  int min_gain_ = 0;
  int max_gain_ = 96;
  int exposure_step_ = 12;
  int gain_step_ = 8;
  double update_interval_ = 0.5;
  double startup_auto_seconds_ = 1.5;
  int set_sharpness_ = 1;
  int power_line_frequency_ = 1;

  int fd_ = -1;
  bool manual_mode_ = false;
  int exposure_ = 166;
  int gain_ = 32;
  rclcpp::Time startup_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V4L2AutoExposureController>());
  rclcpp::shutdown();
  return 0;
}
