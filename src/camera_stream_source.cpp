#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraStreamSource : public rclcpp::Node {
public:
  CameraStreamSource() : Node("camera_stream_source") {
    color_input_topic_ = declare_parameter<std::string>(
        "color_input_topic", "/camera/color/image_raw");
    color_camera_info_input_topic_ = declare_parameter<std::string>(
        "color_camera_info_input_topic", "/camera/color/camera_info");
    depth_input_topic_ = declare_parameter<std::string>(
        "depth_input_topic", "/camera/depth/image_raw");
    depth_camera_info_input_topic_ = declare_parameter<std::string>(
        "depth_camera_info_input_topic", "/camera/depth/camera_info");

    color_output_topic_ =
        declare_parameter<std::string>("color_output_topic", "/vision/color/image_raw");
    color_camera_info_output_topic_ = declare_parameter<std::string>(
        "color_camera_info_output_topic", "/vision/color/camera_info");
    depth_output_topic_ =
        declare_parameter<std::string>("depth_output_topic", "/vision/depth/image_raw");
    depth_camera_info_output_topic_ = declare_parameter<std::string>(
        "depth_camera_info_output_topic", "/vision/depth/camera_info");

    enable_depth_ = declare_parameter<bool>("enable_depth", false);
    enable_stats_ = declare_parameter<bool>("enable_stats", true);
    stats_interval_ = declare_parameter<double>("stats_interval", 5.0);
    image_reliability_ =
        declare_parameter<std::string>("image_reliability", "best_effort");

    const auto image_qos = make_image_qos();

    color_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(color_output_topic_, image_qos);
    color_camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        color_camera_info_output_topic_, rclcpp::QoS(10));

    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_input_topic_, image_qos,
        std::bind(&CameraStreamSource::color_image_callback, this, std::placeholders::_1));
    color_camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        color_camera_info_input_topic_, rclcpp::QoS(10),
        std::bind(&CameraStreamSource::color_camera_info_callback, this,
                  std::placeholders::_1));

    if (enable_depth_) {
      depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
          depth_output_topic_, image_qos);
      depth_camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
          depth_camera_info_output_topic_, rclcpp::QoS(10));

      depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          depth_input_topic_, image_qos,
          std::bind(&CameraStreamSource::depth_image_callback, this, std::placeholders::_1));
      depth_camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          depth_camera_info_input_topic_, rclcpp::QoS(10),
          std::bind(&CameraStreamSource::depth_camera_info_callback, this,
                    std::placeholders::_1));
    }

    if (enable_stats_) {
      stats_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(stats_interval_)),
          std::bind(&CameraStreamSource::print_stats, this));
    }

    RCLCPP_INFO(get_logger(), "Camera stream source started");
    RCLCPP_INFO(get_logger(), "  Color input:  %s", color_input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Color output: %s", color_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Image QoS reliability: %s", image_reliability_.c_str());
    RCLCPP_INFO(get_logger(), "  Depth enabled: %s", enable_depth_ ? "true" : "false");
    if (enable_depth_) {
      RCLCPP_INFO(get_logger(), "  Depth input:  %s", depth_input_topic_.c_str());
      RCLCPP_INFO(get_logger(), "  Depth output: %s", depth_output_topic_.c_str());
    }
  }

private:
  struct Stats {
    std::uint64_t color_images = 0;
    std::uint64_t color_camera_infos = 0;
    std::uint64_t depth_images = 0;
    std::uint64_t depth_camera_infos = 0;
  };

  rclcpp::QoS make_image_qos() const {
    rclcpp::QoS qos(1);
    qos.keep_last(1);
    if (image_reliability_ == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    return qos;
  }

  void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    color_publisher_->publish(*msg);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.color_images++;
  }

  void color_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    color_camera_info_publisher_->publish(*msg);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.color_camera_infos++;
  }

  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!depth_publisher_) {
      return;
    }
    depth_publisher_->publish(*msg);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.depth_images++;
  }

  void depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!depth_camera_info_publisher_) {
      return;
    }
    depth_camera_info_publisher_->publish(*msg);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.depth_camera_infos++;
  }

  void print_stats() {
    Stats snapshot;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      snapshot = stats_;
      stats_ = Stats{};
    }

    if (enable_depth_) {
      RCLCPP_INFO(get_logger(),
                  "Source stats: color_images=%llu color_info=%llu depth_images=%llu "
                  "depth_info=%llu",
                  static_cast<unsigned long long>(snapshot.color_images),
                  static_cast<unsigned long long>(snapshot.color_camera_infos),
                  static_cast<unsigned long long>(snapshot.depth_images),
                  static_cast<unsigned long long>(snapshot.depth_camera_infos));
    } else {
      RCLCPP_INFO(get_logger(), "Source stats: color_images=%llu color_info=%llu",
                  static_cast<unsigned long long>(snapshot.color_images),
                  static_cast<unsigned long long>(snapshot.color_camera_infos));
    }
  }

  std::string color_input_topic_;
  std::string color_camera_info_input_topic_;
  std::string depth_input_topic_;
  std::string depth_camera_info_input_topic_;

  std::string color_output_topic_;
  std::string color_camera_info_output_topic_;
  std::string depth_output_topic_;
  std::string depth_camera_info_output_topic_;

  bool enable_depth_;
  bool enable_stats_;
  double stats_interval_;
  std::string image_reliability_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      color_camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      depth_camera_info_subscription_;

  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::mutex stats_mutex_;
  Stats stats_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraStreamSource>());
  rclcpp::shutdown();
  return 0;
}
