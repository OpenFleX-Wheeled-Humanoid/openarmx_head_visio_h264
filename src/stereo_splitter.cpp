#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class StereoSplitter : public rclcpp::Node {
public:
    StereoSplitter() : Node("stereo_splitter") {
        input_topic_ = declare_parameter<std::string>("input_topic", "/image_raw");
        left_output_topic_ = declare_parameter<std::string>("left_output_topic", "/vision/stereo/left/image_raw");
        right_output_topic_ = declare_parameter<std::string>("right_output_topic", "/vision/stereo/right/image_raw");
        enable_stats_ = declare_parameter<bool>("enable_stats", true);
        stats_interval_ = declare_parameter<double>("stats_interval", 5.0);

        // 订阅输入图像
        auto image_qos = rclcpp::QoS(1).best_effort().keep_last(1);

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic_, image_qos,
            std::bind(&StereoSplitter::image_callback, this, std::placeholders::_1));

        // 发布左右图像
        left_publisher_ = create_publisher<sensor_msgs::msg::Image>(left_output_topic_, image_qos);
        right_publisher_ = create_publisher<sensor_msgs::msg::Image>(right_output_topic_, image_qos);

        if (enable_stats_) {
            stats_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(stats_interval_)),
                std::bind(&StereoSplitter::print_stats, this));
        }

        RCLCPP_INFO(get_logger(), "Stereo splitter started");
        RCLCPP_INFO(get_logger(), "  Input:  %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Left:   %s", left_output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Right:  %s", right_output_topic_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.input_images++;
                stats_.last_width = msg->width;
                stats_.last_height = msg->height;
                stats_.last_encoding = msg->encoding;
            }

            // 转换为 OpenCV 格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

            // 获取图像尺寸
            int width = cv_ptr->image.cols;
            int height = cv_ptr->image.rows;
            int half_width = width / 2;

            // 分割左右图像
            cv::Mat left_image = cv_ptr->image(cv::Rect(0, 0, half_width, height));
            cv::Mat right_image = cv_ptr->image(cv::Rect(half_width, 0, half_width, height));

            // 转换回 ROS 消息
            auto left_msg = cv_bridge::CvImage(msg->header, msg->encoding, left_image).toImageMsg();
            auto right_msg = cv_bridge::CvImage(msg->header, msg->encoding, right_image).toImageMsg();

            // 发布
            left_publisher_->publish(*left_msg);
            right_publisher_->publish(*right_msg);

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.left_images++;
                stats_.right_images++;
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    struct Stats {
        std::uint64_t input_images = 0;
        std::uint64_t left_images = 0;
        std::uint64_t right_images = 0;
        std::uint32_t last_width = 0;
        std::uint32_t last_height = 0;
        std::string last_encoding;
    };

    void print_stats() {
        Stats snapshot;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            snapshot = stats_;
            stats_ = Stats{};
        }
        RCLCPP_INFO(get_logger(),
                    "Splitter stats: input=%llu left=%llu right=%llu last=%ux%u %s",
                    static_cast<unsigned long long>(snapshot.input_images),
                    static_cast<unsigned long long>(snapshot.left_images),
                    static_cast<unsigned long long>(snapshot.right_images),
                    snapshot.last_width,
                    snapshot.last_height,
                    snapshot.last_encoding.c_str());
    }

    std::string input_topic_;
    std::string left_output_topic_;
    std::string right_output_topic_;
    bool enable_stats_;
    double stats_interval_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::mutex stats_mutex_;
    Stats stats_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoSplitter>());
    rclcpp::shutdown();
    return 0;
}
