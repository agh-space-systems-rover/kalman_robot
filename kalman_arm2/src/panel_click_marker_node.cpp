#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace kalman_arm2 {

class PanelClickMarker : public rclcpp::Node {
  public:
    explicit PanelClickMarker(const rclcpp::NodeOptions &options)
        : Node("panel_click_marker", options) {
        board_frame_ = declare_parameter<std::string>(
            "board_frame", "aruco_board"
        );
        const std::string click_topic = declare_parameter<std::string>(
            "click_topic", "panel/image_rectified_mouse_left"
        );
        const std::string transform_topic = declare_parameter<std::string>(
            "pixel_transform_topic", "panel/homography"
        );
        const std::string height_topic = declare_parameter<std::string>(
            "height_topic", "panel/height"
        );
        const std::string valid_topic = declare_parameter<std::string>(
            "valid_topic", "panel/valid"
        );
        const std::string marker_topic = declare_parameter<std::string>(
            "marker_topic", "panel/click_marker"
        );
        marker_diameter_m_ = declare_parameter<double>(
            "marker_diameter_m", 0.025
        );
        marker_z_offset_m_ = declare_parameter<double>(
            "marker_z_offset_m", 0.0
        );
        if (!std::isfinite(marker_diameter_m_) || marker_diameter_m_ <= 0.0 ||
            !std::isfinite(marker_z_offset_m_)) {
            throw std::invalid_argument("invalid click marker dimensions");
        }

        click_sub_ = create_subscription<geometry_msgs::msg::Point>(
            click_topic,
            10,
            std::bind(
                &PanelClickMarker::on_click, this, std::placeholders::_1
            )
        );
        auto transform_qos = rclcpp::QoS(1).reliable().transient_local();
        transform_sub_ =
            create_subscription<std_msgs::msg::Float64MultiArray>(
                transform_topic,
                transform_qos,
                std::bind(
                    &PanelClickMarker::on_transform,
                    this,
                    std::placeholders::_1
                )
            );

        rclcpp::QoS image_qos(1);
        image_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        height_sub_.subscribe(
            this, height_topic, image_qos.get_rmw_qos_profile()
        );
        valid_sub_.subscribe(
            this, valid_topic, image_qos.get_rmw_qos_profile()
        );
        image_sync_ = std::make_shared<ImageSynchronizer>(
            height_sub_, valid_sub_, 5
        );
        image_sync_->registerCallback(std::bind(
            &PanelClickMarker::on_height_and_valid,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            marker_topic, rclcpp::QoS(1).reliable().transient_local()
        );
    }

  private:
    using ImageSynchronizer = message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void on_transform(
        const std_msgs::msg::Float64MultiArray::SharedPtr message
    ) {
        if (message->data.size() != pixel_to_panel_.size()) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Pixel-to-panel transform must contain 9 row-major values, got %zu",
                message->data.size()
            );
            return;
        }
        for (const double value : message->data) {
            if (!std::isfinite(value)) {
                RCLCPP_ERROR(
                    get_logger(), "Pixel-to-panel transform contains non-finite data"
                );
                return;
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        std::copy(
            message->data.begin(), message->data.end(), pixel_to_panel_.begin()
        );
        transform_ready_ = true;
    }

    void on_height_and_valid(
        const sensor_msgs::msg::Image::ConstSharedPtr &height_message,
        const sensor_msgs::msg::Image::ConstSharedPtr &valid_message
    ) {
        try {
            if (height_message->width != valid_message->width ||
                height_message->height != valid_message->height) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Panel height and validity dimensions differ"
                );
                return;
            }
            const auto height = cv_bridge::toCvCopy(
                height_message, sensor_msgs::image_encodings::TYPE_32FC1
            );
            const auto valid = cv_bridge::toCvCopy(
                valid_message, sensor_msgs::image_encodings::MONO8
            );

            std::lock_guard<std::mutex> lock(data_mutex_);
            height_ = height->image.clone();
            valid_ = valid->image.clone();
            images_ready_ = true;
        } catch (const cv_bridge::Exception &error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not consume panel height/valid images: %s",
                error.what()
            );
        }
    }

    void on_click(const geometry_msgs::msg::Point::SharedPtr message) {
        if (!std::isfinite(message->x) || !std::isfinite(message->y)) {
            RCLCPP_WARN(get_logger(), "Ignoring non-finite panel image click");
            return;
        }

        std::array<double, 9> transform;
        cv::Mat height;
        cv::Mat valid;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!transform_ready_ || !images_ready_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Cannot resolve panel click: waiting for transform and height map"
                );
                return;
            }
            transform = pixel_to_panel_;
            height = height_;
            valid = valid_;
        }

        const int pixel_x = static_cast<int>(std::lround(message->x));
        const int pixel_y = static_cast<int>(std::lround(message->y));
        if (pixel_x < 0 || pixel_x >= valid.cols || pixel_y < 0 ||
            pixel_y >= valid.rows) {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring panel click outside image: (%d, %d), image=%dx%d",
                pixel_x,
                pixel_y,
                valid.cols,
                valid.rows
            );
            return;
        }
        if (valid.at<unsigned char>(pixel_y, pixel_x) == 0) {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring panel click at (%d, %d): depth is unknown",
                pixel_x,
                pixel_y
            );
            return;
        }

        const float panel_z = height.at<float>(pixel_y, pixel_x);
        if (!std::isfinite(panel_z)) {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring panel click at (%d, %d): height is non-finite",
                pixel_x,
                pixel_y
            );
            return;
        }

        const double homogeneous_scale =
            transform[6] * message->x + transform[7] * message->y + transform[8];
        if (!std::isfinite(homogeneous_scale) ||
            std::abs(homogeneous_scale) < 1e-12) {
            RCLCPP_ERROR(get_logger(), "Pixel-to-panel transform is singular");
            return;
        }
        const double panel_x =
            (transform[0] * message->x + transform[1] * message->y + transform[2]) /
            homogeneous_scale;
        const double panel_y =
            (transform[3] * message->x + transform[4] * message->y + transform[5]) /
            homogeneous_scale;

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now();
        marker.header.frame_id = board_frame_;
        marker.ns = "panel_click";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = panel_x;
        marker.pose.position.y = panel_y;
        marker.pose.position.z = static_cast<double>(panel_z) + marker_z_offset_m_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker_diameter_m_;
        marker.scale.y = marker_diameter_m_;
        marker.scale.z = marker_diameter_m_;
        marker.color.r = 1.0F;
        marker.color.g = 0.1F;
        marker.color.b = 0.8F;
        marker.color.a = 1.0F;
        marker_pub_->publish(marker);

        RCLCPP_INFO(
            get_logger(),
            "Panel click (%d, %d) -> (%.4f, %.4f, %.4f) m in '%s'",
            pixel_x,
            pixel_y,
            panel_x,
            panel_y,
            static_cast<double>(panel_z),
            board_frame_.c_str()
        );
    }

    std::string board_frame_;
    double marker_diameter_m_{0.025};
    double marker_z_offset_m_{0.0};

    std::mutex data_mutex_;
    std::array<double, 9> pixel_to_panel_{};
    bool transform_ready_{false};
    cv::Mat height_;
    cv::Mat valid_;
    bool images_ready_{false};

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        transform_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> height_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> valid_sub_;
    std::shared_ptr<ImageSynchronizer> image_sync_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelClickMarker)
