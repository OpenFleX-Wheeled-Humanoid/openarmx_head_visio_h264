#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

namespace {

constexpr char kHelloMagic[] = {'O', 'A', 'R', 'H'};
constexpr char kChunkMagic[] = {'O', 'A', 'R', '3'};
constexpr char kFeedbackMagic[] = {'O', 'A', 'R', 'F'};  // 反馈包标识
constexpr uint8_t kProtocolVersion = 1;
constexpr std::size_t kHelloSize = 8;
constexpr std::size_t kFeedbackSize = 32;  // 反馈包大小
constexpr std::size_t kRtpHeaderSize = 12;
constexpr std::size_t kChunkHeaderSize = 32;
constexpr std::size_t kHostMaxDatagramSize = 1200;
constexpr std::size_t kMaxChunkPayloadSize =
    kHostMaxDatagramSize - kRtpHeaderSize - kChunkHeaderSize;
constexpr std::size_t kMaxEncodedPayloadSize = 8 * 1024 * 1024;
constexpr uint8_t kCodecH264 = 1;
constexpr uint8_t kCodecHevc = 2;
constexpr uint8_t kFlagKeyframe = 1U << 0;

enum class VideoCodec {
  H264,
  Hevc,
};

std::string normalize_codec_name(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

const char* codec_label(VideoCodec codec) {
  switch (codec) {
    case VideoCodec::H264:
      return "H264";
    case VideoCodec::Hevc:
      return "HEVC";
  }
  return "Unknown";
}

uint8_t wire_codec_id(VideoCodec codec) {
  switch (codec) {
    case VideoCodec::H264:
      return kCodecH264;
    case VideoCodec::Hevc:
      return kCodecHevc;
  }
  return kCodecH264;
}

void write_big_endian_u16(uint8_t* buffer, std::uint16_t value) {
  buffer[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[1] = static_cast<uint8_t>(value & 0xFF);
}

void write_big_endian_u32(uint8_t* buffer, std::uint32_t value) {
  buffer[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  buffer[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[3] = static_cast<uint8_t>(value & 0xFF);
}

void write_big_endian_u64(uint8_t* buffer, std::uint64_t value) {
  buffer[0] = static_cast<uint8_t>((value >> 56) & 0xFF);
  buffer[1] = static_cast<uint8_t>((value >> 48) & 0xFF);
  buffer[2] = static_cast<uint8_t>((value >> 40) & 0xFF);
  buffer[3] = static_cast<uint8_t>((value >> 32) & 0xFF);
  buffer[4] = static_cast<uint8_t>((value >> 24) & 0xFF);
  buffer[5] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[6] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[7] = static_cast<uint8_t>(value & 0xFF);
}

// 读取大端序数据（用于解析反馈包）
uint32_t read_big_endian_u32(const uint8_t* buffer) {
  return (static_cast<uint32_t>(buffer[0]) << 24) |
         (static_cast<uint32_t>(buffer[1]) << 16) |
         (static_cast<uint32_t>(buffer[2]) << 8) |
         static_cast<uint32_t>(buffer[3]);
}

std::uint64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    return 0;
  }
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nanosec);
}

std::uint64_t now_nanoseconds() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::size_t expected_chunk_count(std::size_t frame_size) {
  return (frame_size + kMaxChunkPayloadSize - 1U) / kMaxChunkPayloadSize;
}

bool same_endpoint(const sockaddr_in& lhs, const sockaddr_in& rhs) {
  return lhs.sin_family == rhs.sin_family && lhs.sin_port == rhs.sin_port &&
         lhs.sin_addr.s_addr == rhs.sin_addr.s_addr;
}

}  // namespace

class VRVideoForwarder : public rclcpp::Node {
public:
  VRVideoForwarder() : Node("vr_video_forwarder") {
    target_fps_ = declare_parameter<int>("target_fps", 20);
    video_bitrate_kbps_ = declare_parameter<int>("video_bitrate_kbps", 4000);
    keyframe_interval_ = declare_parameter<int>("keyframe_interval", 30);
    const int legacy_tcp_port = declare_parameter<int>("tcp_port", 5600);
    const std::string legacy_tcp_host =
        declare_parameter<std::string>("tcp_host", "0.0.0.0");
    const int configured_udp_port = declare_parameter<int>("udp_port", -1);
    const std::string configured_udp_host =
        declare_parameter<std::string>("udp_host", "");
    udp_port_ = configured_udp_port > 0 ? configured_udp_port : legacy_tcp_port;
    udp_host_ = configured_udp_host.empty() ? legacy_tcp_host : configured_udp_host;
    video_codec_name_ =
        normalize_codec_name(declare_parameter<std::string>("video_codec", "h264"));
    if (video_codec_name_ == "h265" || video_codec_name_ == "hevc") {
      video_codec_ = VideoCodec::Hevc;
      video_codec_name_ = "hevc";
    } else if (video_codec_name_ == "h264" || video_codec_name_ == "avc") {
      video_codec_ = VideoCodec::H264;
      video_codec_name_ = "h264";
    } else {
      RCLCPP_WARN(get_logger(), "Unsupported video_codec=%s, falling back to h264",
                  video_codec_name_.c_str());
      video_codec_ = VideoCodec::H264;
      video_codec_name_ = "h264";
    }
    max_queue_size_ = declare_parameter<int>("max_queue_size", 1);
    flip_vertical_ = declare_parameter<bool>("flip_vertical", false);
    flip_horizontal_ = declare_parameter<bool>("flip_horizontal", false);
    side_by_side_per_eye_flip_ = declare_parameter<bool>("side_by_side_per_eye_flip", false);
    side_by_side_swap_eyes_ = declare_parameter<bool>("side_by_side_swap_eyes", false);
    enable_stats_ = declare_parameter<bool>("enable_stats", true);
    stats_interval_ = declare_parameter<double>("stats_interval", 5.0);
    enable_depth_ = declare_parameter<bool>("enable_depth", false);
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_input");
    compressed_image_topic_ =
        declare_parameter<std::string>("compressed_image_topic", image_topic_ + "/compressed");
    prefer_compressed_image_ = declare_parameter<bool>("prefer_compressed_image", false);
    image_reliability_ =
        normalize_codec_name(declare_parameter<std::string>("image_reliability", "best_effort"));
    if (image_reliability_ != "reliable" && image_reliability_ != "best_effort") {
      RCLCPP_WARN(get_logger(), "Unsupported image_reliability=%s, falling back to best_effort",
                  image_reliability_.c_str());
      image_reliability_ = "best_effort";
    }
    frame_interval_ = std::chrono::duration<double>(
        1.0 / static_cast<double>(std::max(target_fps_, 1)));

    if (max_queue_size_ != 1) {
      RCLCPP_WARN(
          get_logger(),
          "max_queue_size=%d requested, but the VR forwarder is latest-frame only. "
          "Using queue size 1 for lower latency.",
          max_queue_size_);
      max_queue_size_ = 1;
    }

    if (prefer_compressed_image_) {
      compressed_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          compressed_image_topic_, make_image_qos(),
          std::bind(&VRVideoForwarder::compressed_image_callback, this,
                    std::placeholders::_1));
    } else {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
          image_topic_, make_image_qos(),
          std::bind(&VRVideoForwarder::image_callback, this, std::placeholders::_1));
    }

    if (enable_stats_) {
      stats_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(stats_interval_)),
          std::bind(&VRVideoForwarder::print_stats, this));
    }

    start_threads();

    RCLCPP_INFO(get_logger(), "VR Video Forwarder (%s/OAR3 UDP) started",
                codec_label(video_codec_));
    RCLCPP_INFO(get_logger(), "  Target FPS: %d", target_fps_);
    RCLCPP_INFO(get_logger(), "  Bitrate: %d kbps", video_bitrate_kbps_);
    RCLCPP_INFO(get_logger(), "  Keyframe interval: %d", keyframe_interval_);
    RCLCPP_INFO(get_logger(), "  UDP endpoint: %s:%d", udp_host_.c_str(), udp_port_);
    RCLCPP_INFO(get_logger(), "  Protocol: OAR3 over RTP-like UDP chunks, codec=%s",
                codec_label(video_codec_));
    RCLCPP_INFO(get_logger(), "  Image input: %s",
                prefer_compressed_image_ ? compressed_image_topic_.c_str()
                                         : image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Latest-frame mode: enabled");
    RCLCPP_INFO(get_logger(), "  Flip vertical: %s", flip_vertical_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Flip horizontal: %s", flip_horizontal_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Side-by-side per-eye flip: %s",
                side_by_side_per_eye_flip_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Side-by-side swap eyes: %s",
                side_by_side_swap_eyes_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Image QoS reliability: %s", image_reliability_.c_str());
    RCLCPP_INFO(get_logger(), "  Depth stream launch flag: %s (VR adapter remains color-only)",
                enable_depth_ ? "true" : "false");
  }

  ~VRVideoForwarder() override {
    running_ = false;
    source_cv_.notify_all();
    encoded_cv_.notify_all();
    close_udp_socket();

    if (hello_thread_.joinable()) {
      hello_thread_.join();
    }
    if (encode_thread_.joinable()) {
      encode_thread_.join();
    }
    if (sender_thread_.joinable()) {
      sender_thread_.join();
    }

    reset_encoder();
  }

private:
  struct SourceFrame {
    cv::Mat image;
    AVPixelFormat pixel_format = AV_PIX_FMT_NONE;
    std::uint64_t timestamp_ns = 0;
  };

  struct EncodedFrame {
    std::vector<uint8_t> payload;
    std::uint64_t timestamp_ns = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t frame_index = 0;
    uint8_t flags = 0;
  };

  struct Stats {
    std::uint64_t frames_received = 0;
    std::uint64_t frames_encoded = 0;
    std::uint64_t frames_sent = 0;
    std::uint64_t frames_dropped = 0;
    std::uint64_t packets_sent = 0;
    std::uint64_t bytes_sent = 0;
    std::uint64_t hellos_received = 0;

    // 自适应码率反馈统计
    std::uint64_t feedbacks_received = 0;
    std::uint32_t client_frames_received = 0;
    std::uint32_t client_frames_decoded = 0;
    std::uint32_t client_chunks_received = 0;
    std::uint32_t client_chunks_lost = 0;
    double packet_loss_rate = 0.0;  // 丢包率
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

  void start_threads() {
    if (!open_udp_socket()) {
      return;
    }
    hello_thread_ = std::thread(&VRVideoForwarder::hello_loop, this);
    encode_thread_ = std::thread(&VRVideoForwarder::encode_loop, this);
    sender_thread_ = std::thread(&VRVideoForwarder::sender_loop, this);
  }

  bool open_udp_socket() {
    udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create UDP socket: %s", std::strerror(errno));
      return false;
    }

    const int reuse = 1;
    ::setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const int sndbuf_size = 1024 * 1024;
    ::setsockopt(udp_socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size));
    const int rcvbuf_size = 128 * 1024;
    ::setsockopt(udp_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    ::setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (udp_host_ == "0.0.0.0") {
      address.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, udp_host_.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid UDP host: %s", udp_host_.c_str());
      close_udp_socket();
      return false;
    }

    if (::bind(udp_socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind UDP server %s:%d: %s",
                   udp_host_.c_str(), udp_port_, std::strerror(errno));
      close_udp_socket();
      return false;
    }

    RCLCPP_INFO(get_logger(), "UDP server listening on %s:%d", udp_host_.c_str(), udp_port_);
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.frames_received++;
    }

    if (!has_recent_client()) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(rate_mutex_);
      if (now - last_accepted_frame_time_ < frame_interval_) {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.frames_dropped++;
        return;
      }
      last_accepted_frame_time_ = now;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    AVPixelFormat source_pixel_format = AV_PIX_FMT_NONE;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv_ptr = cv_bridge::toCvShare(msg);
        source_pixel_format = AV_PIX_FMT_RGB24;
      } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv_ptr = cv_bridge::toCvShare(msg);
        source_pixel_format = AV_PIX_FMT_BGR24;
      } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv_ptr = cv_bridge::toCvShare(msg);
        source_pixel_format = AV_PIX_FMT_GRAY8;
      } else {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        source_pixel_format = AV_PIX_FMT_BGR24;
      }
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Failed to access/convert image: %s", ex.what());
      return;
    }

    SourceFrame frame;
    frame.image = cv_ptr->image.clone();
    frame.pixel_format = source_pixel_format;
    frame.timestamp_ns = stamp_to_ns(msg->header.stamp);
    if (frame.timestamp_ns == 0) {
      frame.timestamp_ns = now_nanoseconds();
    }

    {
      std::lock_guard<std::mutex> lock(source_mutex_);
      latest_source_frame_ = std::move(frame);
      has_source_frame_ = true;
    }
    source_cv_.notify_one();
  }

  void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.frames_received++;
    }

    if (!has_recent_client()) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(rate_mutex_);
      if (now - last_accepted_frame_time_ < frame_interval_) {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.frames_dropped++;
        return;
      }
      last_accepted_frame_time_ = now;
    }

    if (msg->data.empty()) {
      return;
    }

    cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                    const_cast<std::uint8_t*>(msg->data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Failed to decode compressed image (%s)",
                            msg->format.c_str());
      return;
    }

    SourceFrame frame;
    frame.image = std::move(image);
    frame.pixel_format = AV_PIX_FMT_BGR24;
    frame.timestamp_ns = stamp_to_ns(msg->header.stamp);
    if (frame.timestamp_ns == 0) {
      frame.timestamp_ns = now_nanoseconds();
    }

    {
      std::lock_guard<std::mutex> lock(source_mutex_);
      latest_source_frame_ = std::move(frame);
      has_source_frame_ = true;
    }
    source_cv_.notify_one();
  }

  void hello_loop() {
    while (running_) {
      uint8_t packet[256] = {0};
      sockaddr_in remote_addr{};
      socklen_t remote_len = sizeof(remote_addr);
      const ssize_t received = ::recvfrom(
          udp_socket_, packet, sizeof(packet), 0,
          reinterpret_cast<sockaddr*>(&remote_addr), &remote_len);

      if (received < 0) {
        if (!running_) {
          break;
        }
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "recvfrom() failed: %s", std::strerror(errno));
        continue;
      }

      // 检查是否是 Hello 包
      if (static_cast<std::size_t>(received) >= kHelloSize &&
          std::memcmp(packet, kHelloMagic, sizeof(kHelloMagic)) == 0 &&
          packet[4] == kProtocolVersion) {
        // 处理 Hello 包
        handle_hello_packet(remote_addr);
        continue;
      }

      // 检查是否是反馈包（自适应码率）
      if (static_cast<std::size_t>(received) >= kFeedbackSize &&
          std::memcmp(packet, kFeedbackMagic, sizeof(kFeedbackMagic)) == 0 &&
          packet[4] == kProtocolVersion) {
        // 处理反馈包
        handle_feedback_packet(packet);
        continue;
      }
    }
  }

  void handle_hello_packet(const sockaddr_in& remote_addr) {
    bool new_client = false;
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      new_client = !has_client_address_ || !same_endpoint(client_address_, remote_addr);
      client_address_ = remote_addr;
      has_client_address_ = true;
      last_client_hello_time_ = std::chrono::steady_clock::now();
    }

    client_connected_.store(true);
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.hellos_received++;
    }
    if (new_client) {
      clear_encoded_queue();
      encoder_resync_requested_.store(true);
      char client_ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &remote_addr.sin_addr, client_ip, sizeof(client_ip));
      RCLCPP_INFO(get_logger(), "VR UDP client registered from %s:%u", client_ip,
                  ntohs(remote_addr.sin_port));
    }
  }

  void handle_feedback_packet(const uint8_t* packet) {
    // 解析反馈包
    uint32_t client_frames_received = read_big_endian_u32(packet + 8);
    uint32_t client_frames_decoded = read_big_endian_u32(packet + 12);
    uint32_t client_chunks_received = read_big_endian_u32(packet + 16);
    uint32_t client_chunks_lost = read_big_endian_u32(packet + 20);

    // 计算丢包率
    double packet_loss_rate = 0.0;
    if (client_chunks_received + client_chunks_lost > 0) {
      packet_loss_rate = static_cast<double>(client_chunks_lost) /
                        (client_chunks_received + client_chunks_lost) * 100.0;
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.feedbacks_received++;
      stats_.client_frames_received = client_frames_received;
      stats_.client_frames_decoded = client_frames_decoded;
      stats_.client_chunks_received = client_chunks_received;
      stats_.client_chunks_lost = client_chunks_lost;
      stats_.packet_loss_rate = packet_loss_rate;
    }

    // 自适应码率调整
    adjust_bitrate_based_on_feedback(packet_loss_rate, client_frames_decoded);
  }

  void adjust_bitrate_based_on_feedback(double packet_loss_rate, uint32_t frames_decoded) {
    // 自适应码率策略：
    // - 丢包率 < 1%:  增加码率 10%
    // - 丢包率 1-5%:  保持不变
    // - 丢包率 5-10%: 降低码率 10%
    // - 丢包率 > 10%: 降低码率 20%

    int current_bitrate = video_bitrate_kbps_;
    int new_bitrate = current_bitrate;

    if (packet_loss_rate < 1.0 && frames_decoded >= 15) {
      // 网络良好，尝试提高码率
      new_bitrate = static_cast<int>(current_bitrate * 1.1);
      new_bitrate = std::min(new_bitrate, 8000);  // 最高 8Mbps
    } else if (packet_loss_rate > 10.0) {
      // 严重丢包，大幅降低码率
      new_bitrate = static_cast<int>(current_bitrate * 0.8);
      new_bitrate = std::max(new_bitrate, 1000);  // 最低 1Mbps
    } else if (packet_loss_rate > 5.0) {
      // 中等丢包，小幅降低码率
      new_bitrate = static_cast<int>(current_bitrate * 0.9);
      new_bitrate = std::max(new_bitrate, 1000);
    }

    if (new_bitrate != current_bitrate) {
      video_bitrate_kbps_ = new_bitrate;
      encoder_resync_requested_.store(true);  // 重新初始化编码器

      RCLCPP_INFO(get_logger(),
                  "Adaptive bitrate: %.1f%% loss → %d kbps (was %d kbps)",
                  packet_loss_rate, new_bitrate, current_bitrate);
    }
  }

  bool has_recent_client() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!has_client_address_) {
      client_connected_.store(false);
      return false;
    }
    const auto age = std::chrono::steady_clock::now() - last_client_hello_time_;
    const bool active = age < std::chrono::seconds(3);
    client_connected_.store(active);
    return active;
  }

  void encode_loop() {
    while (running_) {
      SourceFrame frame;
      {
        std::unique_lock<std::mutex> lock(source_mutex_);
        source_cv_.wait(lock, [this]() { return has_source_frame_ || !running_; });
        if (!running_) {
          break;
        }

        frame = std::move(latest_source_frame_);
        latest_source_frame_ = SourceFrame{};
        has_source_frame_ = false;
      }

      if (frame.image.empty()) {
        continue;
      }

      apply_frame_orientation(frame.image);

      if (encoder_resync_requested_.exchange(false)) {
        reset_encoder();
      }

      if (!ensure_encoder(frame.image.cols, frame.image.rows, frame.pixel_format)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Failed to initialize %s encoder",
                              codec_label(video_codec_));
        continue;
      }

      if (!encode_frame(frame)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Failed to encode %s frame", codec_label(video_codec_));
      }
    }
  }

  bool encode_frame(const SourceFrame& source) {
    if (encoder_context_ == nullptr || sws_context_ == nullptr || yuv_frame_ == nullptr ||
        packet_ == nullptr) {
      return false;
    }

    if (av_frame_make_writable(yuv_frame_) < 0) {
      return false;
    }

    const uint8_t* src_slices[1] = {source.image.data};
    const int src_stride[1] = {static_cast<int>(source.image.step)};
    sws_scale(sws_context_, src_slices, src_stride, 0, encoder_height_, yuv_frame_->data,
              yuv_frame_->linesize);

    yuv_frame_->pts = next_pts_++;

    if (avcodec_send_frame(encoder_context_, yuv_frame_) < 0) {
      return false;
    }

    bool produced_packet = false;
    EncodedFrame encoded;
    encoded.timestamp_ns = source.timestamp_ns;
    encoded.width = static_cast<std::uint32_t>(encoder_width_);
    encoded.height = static_cast<std::uint32_t>(encoder_height_);
    encoded.frame_index = frame_index_++;
    while (running_) {
      const int result = avcodec_receive_packet(encoder_context_, packet_);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
        break;
      }
      if (result < 0) {
        return false;
      }

      encoded.payload.insert(encoded.payload.end(), packet_->data, packet_->data + packet_->size);
      if (packet_->flags & AV_PKT_FLAG_KEY) {
        encoded.flags |= kFlagKeyframe;
      }
      produced_packet = true;
      av_packet_unref(packet_);
    }

    if (!produced_packet) {
      return true;
    }
    if (encoded.payload.empty() || encoded.payload.size() > kMaxEncodedPayloadSize) {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.frames_dropped++;
      return true;
    }

    enqueue_encoded_frame(std::move(encoded));
    return produced_packet;
  }

  void apply_frame_orientation(cv::Mat& image) {
    if (!flip_vertical_ && !flip_horizontal_ && !side_by_side_swap_eyes_) {
      return;
    }

    if (side_by_side_per_eye_flip_ || side_by_side_swap_eyes_) {
      if (image.cols < 2 || (image.cols % 2) != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "side-by-side eye transform requested but image width=%d is invalid",
                             image.cols);
        return;
      }
      const int half_width = image.cols / 2;
      cv::Mat left = image(cv::Rect(0, 0, half_width, image.rows));
      cv::Mat right = image(cv::Rect(half_width, 0, half_width, image.rows));
      if (flip_vertical_ || flip_horizontal_) {
        const int flip_code = flip_vertical_ && flip_horizontal_ ? -1 : (flip_vertical_ ? 0 : 1);
        cv::flip(left, left, flip_code);
        cv::flip(right, right, flip_code);
      }
      if (side_by_side_swap_eyes_) {
        cv::Mat left_copy = left.clone();
        right.copyTo(left);
        left_copy.copyTo(right);
      }
      return;
    }

    if (flip_vertical_ && flip_horizontal_) {
      cv::flip(image, image, -1);
    } else if (flip_vertical_) {
      cv::flip(image, image, 0);
    } else if (flip_horizontal_) {
      cv::flip(image, image, 1);
    }
  }

  void enqueue_encoded_frame(EncodedFrame&& encoded) {
    bool should_enqueue = true;
    std::size_t dropped_frames = 0;
    {
      std::lock_guard<std::mutex> lock(encoded_mutex_);
      if (encoded_queue_.size() >= kMaxEncodedQueueDepth) {
        dropped_frames = encoded_queue_.size() + 1;
        encoded_queue_.clear();
        should_enqueue = false;
      } else {
        encoded_queue_.push_back(std::move(encoded));
      }
    }

    if (dropped_frames > 0) {
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_dropped += dropped_frames;
      }
      encoder_resync_requested_.store(true);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Encoded queue overflowed; dropped %zu frame(s) and requested encoder resync",
                           dropped_frames);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.frames_encoded++;
    }
    if (should_enqueue) {
      encoded_cv_.notify_one();
    }
  }

  bool ensure_encoder(int width, int height, AVPixelFormat source_pixel_format) {
    if (encoder_context_ != nullptr && width == encoder_width_ && height == encoder_height_ &&
        source_pixel_format == source_pixel_format_) {
      return true;
    }

    reset_encoder();

    const AVCodec* codec = nullptr;
    if (video_codec_ == VideoCodec::H264) {
      codec = avcodec_find_encoder_by_name("libx264");
      if (codec == nullptr) {
        codec = avcodec_find_encoder_by_name("h264_v4l2m2m");
      }
      if (codec == nullptr) {
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
      }
    } else {
      codec = avcodec_find_encoder_by_name("libx265");
      if (codec == nullptr) {
        codec = avcodec_find_encoder_by_name("hevc_v4l2m2m");
      }
      if (codec == nullptr) {
        codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
      }
    }
    if (codec == nullptr) {
      RCLCPP_ERROR(get_logger(), "No %s encoder available on this system",
                   codec_label(video_codec_));
      return false;
    }

    encoder_context_ = avcodec_alloc_context3(codec);
    if (encoder_context_ == nullptr) {
      return false;
    }

    encoder_width_ = width;
    encoder_height_ = height;
    source_pixel_format_ = source_pixel_format;
    encoder_context_->codec_type = AVMEDIA_TYPE_VIDEO;
    encoder_context_->codec_id = codec->id;
    encoder_context_->width = width;
    encoder_context_->height = height;
    encoder_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    encoder_context_->bit_rate = static_cast<int64_t>(video_bitrate_kbps_) * 1000LL;
    encoder_context_->rc_max_rate = encoder_context_->bit_rate;
    encoder_context_->rc_buffer_size = encoder_context_->bit_rate;
    encoder_context_->gop_size = std::max(keyframe_interval_, 1);
    encoder_context_->max_b_frames = 0;
    encoder_context_->thread_count = 1;
    encoder_context_->time_base = AVRational{1, std::max(target_fps_, 1)};
    encoder_context_->framerate = AVRational{std::max(target_fps_, 1), 1};
    encoder_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;

    if (encoder_context_->priv_data != nullptr) {
      av_opt_set(encoder_context_->priv_data, "preset", "ultrafast", 0);
      av_opt_set(encoder_context_->priv_data, "tune", "zerolatency", 0);
      if (video_codec_ == VideoCodec::H264) {
        av_opt_set(encoder_context_->priv_data, "profile", "baseline", 0);
      }
      av_opt_set(encoder_context_->priv_data, "repeat-headers", "1", 0);
      av_opt_set(encoder_context_->priv_data, "annexb", "1", 0);
    }

    if (avcodec_open2(encoder_context_, codec, nullptr) < 0) {
      reset_encoder();
      return false;
    }

    yuv_frame_ = av_frame_alloc();
    if (yuv_frame_ == nullptr) {
      reset_encoder();
      return false;
    }
    yuv_frame_->format = encoder_context_->pix_fmt;
    yuv_frame_->width = encoder_width_;
    yuv_frame_->height = encoder_height_;
    if (av_frame_get_buffer(yuv_frame_, 32) < 0) {
      reset_encoder();
      return false;
    }

    packet_ = av_packet_alloc();
    if (packet_ == nullptr) {
      reset_encoder();
      return false;
    }

    sws_context_ = sws_getCachedContext(nullptr, encoder_width_, encoder_height_,
                                        source_pixel_format_, encoder_width_, encoder_height_,
                                        AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr,
                                        nullptr);
    if (sws_context_ == nullptr) {
      reset_encoder();
      return false;
    }

    next_pts_ = 0;
    frame_index_ = 0;
    RCLCPP_INFO(get_logger(), "%s encoder ready: %dx%d @ %dfps, %dkbps",
                codec_label(video_codec_), encoder_width_, encoder_height_, target_fps_,
                video_bitrate_kbps_);
    return true;
  }

  void reset_encoder() {
    if (packet_ != nullptr) {
      av_packet_free(&packet_);
    }
    if (yuv_frame_ != nullptr) {
      av_frame_free(&yuv_frame_);
    }
    if (encoder_context_ != nullptr) {
      avcodec_free_context(&encoder_context_);
    }
    if (sws_context_ != nullptr) {
      sws_freeContext(sws_context_);
      sws_context_ = nullptr;
    }

    encoder_width_ = 0;
    encoder_height_ = 0;
    source_pixel_format_ = AV_PIX_FMT_NONE;
    next_pts_ = 0;
    frame_index_ = 0;
  }

  void sender_loop() {
    while (running_) {
      EncodedFrame frame;
      {
        std::unique_lock<std::mutex> lock(encoded_mutex_);
        encoded_cv_.wait(lock, [this]() { return !encoded_queue_.empty() || !running_; });
        if (!running_) {
          break;
        }
        frame = std::move(encoded_queue_.front());
        encoded_queue_.pop_front();
      }

      if (!has_recent_client()) {
        clear_encoded_queue();
        continue;
      }

      sockaddr_in client_address{};
      {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (!has_client_address_) {
          continue;
        }
        client_address = client_address_;
      }

      if (!send_encoded_frame(frame, client_address)) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_dropped++;
      }
    }
  }

  bool send_encoded_frame(const EncodedFrame& frame, const sockaddr_in& client_address) {
    if (udp_socket_ < 0 || frame.payload.empty() || frame.payload.size() > kMaxEncodedPayloadSize ||
        frame.width == 0 || frame.height == 0 || frame.width > 65535 || frame.height > 65535) {
      return false;
    }

    const std::size_t chunk_count_size = expected_chunk_count(frame.payload.size());
    if (chunk_count_size == 0 || chunk_count_size > 65535) {
      return false;
    }
    const auto chunk_count = static_cast<std::uint16_t>(chunk_count_size);

    std::uint64_t frame_bytes_sent = 0;
    for (std::uint16_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
      const std::size_t payload_offset =
          static_cast<std::size_t>(chunk_index) * kMaxChunkPayloadSize;
      const std::size_t chunk_payload_size =
          std::min(kMaxChunkPayloadSize, frame.payload.size() - payload_offset);

      uint8_t packet[kHostMaxDatagramSize] = {0};
      packet[0] = 0x80;
      packet[1] = static_cast<uint8_t>((chunk_index + 1 == chunk_count ? 0x80 : 0x00) | 96);
      write_big_endian_u16(packet + 2, rtp_sequence_++);
      write_big_endian_u32(packet + 4, frame.frame_index);
      write_big_endian_u32(packet + 8, 0x4F415833U);

      uint8_t* chunk_header = packet + kRtpHeaderSize;
      std::memcpy(chunk_header, kChunkMagic, sizeof(kChunkMagic));
      chunk_header[4] = kProtocolVersion;
      chunk_header[5] = wire_codec_id(video_codec_);
      chunk_header[6] = frame.flags;
      write_big_endian_u32(chunk_header + 8, frame.frame_index);
      write_big_endian_u32(chunk_header + 12, static_cast<std::uint32_t>(frame.payload.size()));
      write_big_endian_u16(chunk_header + 16, chunk_index);
      write_big_endian_u16(chunk_header + 18, chunk_count);
      write_big_endian_u16(chunk_header + 20, static_cast<std::uint16_t>(frame.width));
      write_big_endian_u16(chunk_header + 22, static_cast<std::uint16_t>(frame.height));
      write_big_endian_u64(chunk_header + 24, frame.timestamp_ns);

      std::memcpy(packet + kRtpHeaderSize + kChunkHeaderSize,
                  frame.payload.data() + payload_offset, chunk_payload_size);
      const std::size_t packet_size = kRtpHeaderSize + kChunkHeaderSize + chunk_payload_size;
      const ssize_t sent = ::sendto(udp_socket_, packet, packet_size, 0,
                                    reinterpret_cast<const sockaddr*>(&client_address),
                                    sizeof(client_address));
      if (sent < 0 || static_cast<std::size_t>(sent) != packet_size) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "sendto() failed while sending video frame: %s",
                             std::strerror(errno));
        return false;
      }
      frame_bytes_sent += static_cast<std::uint64_t>(sent);
    }

    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.frames_sent++;
    stats_.packets_sent += chunk_count;
    stats_.bytes_sent += frame_bytes_sent;
    return true;
  }

  void clear_encoded_queue() {
    std::lock_guard<std::mutex> lock(encoded_mutex_);
    encoded_queue_.clear();
  }

  void print_stats() {
    Stats snapshot;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      snapshot = stats_;
      stats_.bytes_sent = 0;
    }

    const double sent_mbps =
        (snapshot.bytes_sent * 8.0) / (1024.0 * 1024.0 * std::max(stats_interval_, 0.001));

    if (snapshot.feedbacks_received > 0) {
      // 有反馈包时，显示详细的自适应码率信息
      RCLCPP_INFO(get_logger(),
                  "Stats: Recv=%llu Enc=%llu Sent=%llu Pkts=%llu Drop=%llu Hello=%llu | "
                  "BW=%.2fMbps Client=%s | "
                  "Feedback: VR_Recv=%u VR_Dec=%u Loss=%.1f%% | Bitrate=%dkbps",
                  static_cast<unsigned long long>(snapshot.frames_received),
                  static_cast<unsigned long long>(snapshot.frames_encoded),
                  static_cast<unsigned long long>(snapshot.frames_sent),
                  static_cast<unsigned long long>(snapshot.packets_sent),
                  static_cast<unsigned long long>(snapshot.frames_dropped),
                  static_cast<unsigned long long>(snapshot.hellos_received),
                  sent_mbps,
                  has_recent_client() ? "✓" : "✗",
                  snapshot.client_frames_received,
                  snapshot.client_frames_decoded,
                  snapshot.packet_loss_rate,
                  video_bitrate_kbps_);
    } else {
      // 没有反馈包时，显示基本信息
      RCLCPP_INFO(get_logger(),
                  "Stats: Recv=%llu Enc=%llu Sent=%llu Pkts=%llu Drop=%llu Hello=%llu | "
                  "BW=%.2fMbps Client=%s",
                  static_cast<unsigned long long>(snapshot.frames_received),
                  static_cast<unsigned long long>(snapshot.frames_encoded),
                  static_cast<unsigned long long>(snapshot.frames_sent),
                  static_cast<unsigned long long>(snapshot.packets_sent),
                  static_cast<unsigned long long>(snapshot.frames_dropped),
                  static_cast<unsigned long long>(snapshot.hellos_received),
                  sent_mbps,
                  has_recent_client() ? "✓" : "✗");
    }
  }

  void close_udp_socket() {
    if (udp_socket_ >= 0) {
      ::shutdown(udp_socket_, SHUT_RDWR);
      ::close(udp_socket_);
      udp_socket_ = -1;
    }
    client_connected_.store(false);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  int target_fps_ = 20;
  int video_bitrate_kbps_ = 4000;
  int keyframe_interval_ = 30;
  int udp_port_ = 5600;
  std::string udp_host_;
  VideoCodec video_codec_ = VideoCodec::H264;
  std::string video_codec_name_ = "h264";
  int max_queue_size_ = 1;
  bool flip_vertical_ = false;
  bool flip_horizontal_ = false;
  bool side_by_side_per_eye_flip_ = false;
  bool side_by_side_swap_eyes_ = false;
  bool enable_stats_ = true;
  double stats_interval_ = 5.0;
  bool enable_depth_ = false;
  std::string image_topic_ = "/image_input";
  std::string compressed_image_topic_ = "/image_input/compressed";
  bool prefer_compressed_image_ = false;
  std::string image_reliability_ = "best_effort";

  std::chrono::duration<double> frame_interval_{0.05};
  std::mutex rate_mutex_;
  std::chrono::steady_clock::time_point last_accepted_frame_time_{};

  std::atomic<bool> running_{true};
  std::atomic<bool> client_connected_{false};

  int udp_socket_ = -1;
  std::mutex client_mutex_;
  sockaddr_in client_address_{};
  bool has_client_address_ = false;
  std::chrono::steady_clock::time_point last_client_hello_time_{};
  std::uint16_t rtp_sequence_ = 0;

  std::thread hello_thread_;
  std::thread encode_thread_;
  std::thread sender_thread_;

  std::mutex source_mutex_;
  std::condition_variable source_cv_;
  SourceFrame latest_source_frame_;
  bool has_source_frame_ = false;

  std::mutex encoded_mutex_;
  std::condition_variable encoded_cv_;
  static constexpr std::size_t kMaxEncodedQueueDepth = 2;
  std::deque<EncodedFrame> encoded_queue_;

  std::mutex stats_mutex_;
  Stats stats_;

  AVCodecContext* encoder_context_ = nullptr;
  SwsContext* sws_context_ = nullptr;
  AVFrame* yuv_frame_ = nullptr;
  AVPacket* packet_ = nullptr;
  int encoder_width_ = 0;
  int encoder_height_ = 0;
  AVPixelFormat source_pixel_format_ = AV_PIX_FMT_NONE;
  std::uint32_t frame_index_ = 0;
  std::int64_t next_pts_ = 0;
  std::atomic<bool> encoder_resync_requested_{true};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VRVideoForwarder>());
  rclcpp::shutdown();
  return 0;
}
