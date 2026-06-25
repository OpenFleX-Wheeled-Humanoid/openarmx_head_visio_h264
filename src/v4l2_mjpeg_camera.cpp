#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
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

std::string fourcc_to_string(std::uint32_t fourcc) {
  std::string value(4, ' ');
  value[0] = static_cast<char>(fourcc & 0xFF);
  value[1] = static_cast<char>((fourcc >> 8) & 0xFF);
  value[2] = static_cast<char>((fourcc >> 16) & 0xFF);
  value[3] = static_cast<char>((fourcc >> 24) & 0xFF);
  return value;
}

}  // namespace

class V4L2MjpegCamera : public rclcpp::Node {
public:
  V4L2MjpegCamera() : Node("v4l2_mjpeg_camera") {
    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    output_topic_ = declare_parameter<std::string>("output_topic", "/image_raw");
    compressed_output_topic_ =
        declare_parameter<std::string>("compressed_output_topic", "/image_raw/compressed");
    frame_id_ = declare_parameter<std::string>("frame_id", "stereo_camera");
    image_width_ = declare_parameter<int>("image_width", 1280);
    image_height_ = declare_parameter<int>("image_height", 480);
    camera_fps_ = declare_parameter<int>("camera_fps", 30);
    buffer_count_ = declare_parameter<int>("buffer_count", 4);
    publish_raw_ = declare_parameter<bool>("publish_raw", false);
    publish_compressed_ = declare_parameter<bool>("publish_compressed", true);
    enable_stats_ = declare_parameter<bool>("enable_stats", true);
    stats_interval_ = declare_parameter<double>("stats_interval", 5.0);

    if (image_width_ <= 0 || image_height_ <= 0) {
      throw std::runtime_error("image_width and image_height must be positive");
    }
    if (camera_fps_ <= 0) {
      throw std::runtime_error("camera_fps must be positive");
    }
    buffer_count_ = std::max(2, buffer_count_);

    if (!publish_raw_ && !publish_compressed_) {
      throw std::runtime_error("At least one of publish_raw/publish_compressed must be true");
    }

    if (publish_raw_) {
      raw_publisher_ = create_publisher<sensor_msgs::msg::Image>(
          output_topic_, rclcpp::QoS(1).best_effort().keep_last(1));
    }
    if (publish_compressed_) {
      compressed_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          compressed_output_topic_, rclcpp::QoS(1).best_effort().keep_last(1));
    }

    open_device();
    configure_device();
    init_mmap();
    start_streaming();

    capture_thread_ = std::thread(&V4L2MjpegCamera::capture_loop, this);

    if (enable_stats_) {
      stats_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(stats_interval_)),
          std::bind(&V4L2MjpegCamera::print_stats, this));
    }

    RCLCPP_INFO(get_logger(),
                "V4L2 MJPEG camera started: %s %dx%d @ %dfps -> raw=%s compressed=%s",
                video_device_.c_str(), image_width_, image_height_, camera_fps_,
                publish_raw_ ? output_topic_.c_str() : "disabled",
                publish_compressed_ ? compressed_output_topic_.c_str() : "disabled");
  }

  ~V4L2MjpegCamera() override {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    stop_streaming();
    unmap_buffers();
    close_device();
  }

private:
  struct Buffer {
    void* start = nullptr;
    std::size_t length = 0;
  };

  struct Stats {
    std::uint64_t captured = 0;
    std::uint64_t decoded = 0;
    std::uint64_t raw_published = 0;
    std::uint64_t compressed_published = 0;
    std::uint64_t decode_failed = 0;
    std::uint64_t dequeue_failed = 0;
  };

  void open_device() {
    fd_ = ::open(video_device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + video_device_ + ": " + std::strerror(errno));
    }
  }

  void configure_device() {
    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
      throw std::runtime_error("VIDIOC_QUERYCAP failed: " + std::string(std::strerror(errno)));
    }
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
        (cap.capabilities & V4L2_CAP_STREAMING) == 0) {
      throw std::runtime_error(video_device_ + " is not a streaming capture device");
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<std::uint32_t>(image_width_);
    fmt.fmt.pix.height = static_cast<std::uint32_t>(image_height_);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT MJPG failed: " + std::string(std::strerror(errno)));
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG ||
        static_cast<int>(fmt.fmt.pix.width) != image_width_ ||
        static_cast<int>(fmt.fmt.pix.height) != image_height_) {
      throw std::runtime_error(
          "Camera did not accept requested MJPG format. Got " +
          std::to_string(fmt.fmt.pix.width) + "x" + std::to_string(fmt.fmt.pix.height) +
          " " + fourcc_to_string(fmt.fmt.pix.pixelformat));
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(camera_fps_);
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_S_PARM failed: %s", std::strerror(errno));
    }

    const auto numerator = parm.parm.capture.timeperframe.numerator;
    const auto denominator = parm.parm.capture.timeperframe.denominator;
    if (numerator > 0 && denominator > 0) {
      RCLCPP_INFO(get_logger(), "Camera accepted frame interval %u/%u sec",
                  numerator, denominator);
    }
  }

  void init_mmap() {
    v4l2_requestbuffers req{};
    req.count = static_cast<std::uint32_t>(buffer_count_);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(std::strerror(errno)));
    }
    if (req.count < 2) {
      throw std::runtime_error("Insufficient V4L2 mmap buffers");
    }

    buffers_.resize(req.count);
    for (std::uint32_t i = 0; i < req.count; ++i) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(std::strerror(errno)));
      }

      buffers_[i].length = buf.length;
      buffers_[i].start =
          ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
      }
    }
  }

  void start_streaming() {
    for (std::uint32_t i = 0; i < buffers_.size(); ++i) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(std::strerror(errno)));
    }
    streaming_ = true;
  }

  void stop_streaming() {
    if (!streaming_ || fd_ < 0) {
      return;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
      RCLCPP_WARN(get_logger(), "VIDIOC_STREAMOFF failed: %s", std::strerror(errno));
    }
    streaming_ = false;
  }

  void unmap_buffers() {
    for (auto& buffer : buffers_) {
      if (buffer.start != nullptr) {
        ::munmap(buffer.start, buffer.length);
        buffer.start = nullptr;
      }
    }
    buffers_.clear();
  }

  void close_device() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void capture_loop() {
    running_ = true;
    while (rclcpp::ok() && running_) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(fd_, &fds);
      timeval timeout{};
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;

      const int selected = ::select(fd_ + 1, &fds, nullptr, nullptr, &timeout);
      if (selected < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "select failed: %s", std::strerror(errno));
        continue;
      }
      if (selected == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Timed out waiting for V4L2 frame");
        continue;
      }

      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(stats_mutex_);
          stats_.dequeue_failed++;
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "VIDIOC_DQBUF failed: %s", std::strerror(errno));
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.captured++;
      }

      if (buf.index < buffers_.size()) {
        publish_mjpeg_frame(static_cast<const std::uint8_t*>(buffers_[buf.index].start),
                            buf.bytesused);
      }

      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "VIDIOC_QBUF requeue failed: %s", std::strerror(errno));
      }
    }
  }

  void publish_mjpeg_frame(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return;
    }

    const auto stamp = now();

    if (compressed_publisher_ != nullptr) {
      auto msg = sensor_msgs::msg::CompressedImage();
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.format = "jpeg";
      msg.data.assign(data, data + size);
      compressed_publisher_->publish(std::move(msg));

      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.compressed_published++;
    }

    if (raw_publisher_ == nullptr) {
      return;
    }

    const cv::Mat encoded(1, static_cast<int>(size), CV_8UC1,
                          const_cast<std::uint8_t*>(data));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.decode_failed++;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.decoded++;
    }

    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<std::uint32_t>(image.rows);
    msg.width = static_cast<std::uint32_t>(image.cols);
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
    msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);
    if (image.isContinuous()) {
      std::memcpy(msg.data.data(), image.data, msg.data.size());
    } else {
      for (int row = 0; row < image.rows; ++row) {
        std::memcpy(msg.data.data() + static_cast<std::size_t>(row) * msg.step,
                    image.ptr(row), msg.step);
      }
    }

    raw_publisher_->publish(std::move(msg));

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.raw_published++;
    }
  }

  void print_stats() {
    Stats snapshot;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      snapshot = stats_;
      stats_ = Stats{};
    }

    RCLCPP_INFO(get_logger(),
                "V4L2 camera stats: captured=%llu compressed=%llu decoded=%llu raw=%llu "
                "decode_fail=%llu dq_fail=%llu",
                static_cast<unsigned long long>(snapshot.captured),
                static_cast<unsigned long long>(snapshot.compressed_published),
                static_cast<unsigned long long>(snapshot.decoded),
                static_cast<unsigned long long>(snapshot.raw_published),
                static_cast<unsigned long long>(snapshot.decode_failed),
                static_cast<unsigned long long>(snapshot.dequeue_failed));
  }

  std::string video_device_;
  std::string output_topic_;
  std::string compressed_output_topic_;
  std::string frame_id_;
  int image_width_ = 1280;
  int image_height_ = 480;
  int camera_fps_ = 30;
  int buffer_count_ = 4;
  bool publish_raw_ = false;
  bool publish_compressed_ = true;
  bool enable_stats_ = true;
  double stats_interval_ = 5.0;

  int fd_ = -1;
  bool streaming_ = false;
  std::atomic<bool> running_{false};
  std::vector<Buffer> buffers_;
  std::thread capture_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::mutex stats_mutex_;
  Stats stats_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V4L2MjpegCamera>());
  rclcpp::shutdown();
  return 0;
}
