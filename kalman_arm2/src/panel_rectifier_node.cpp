#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace kalman_arm2 {

class PanelRectifier : public rclcpp::Node {
  public:
    explicit PanelRectifier(const rclcpp::NodeOptions &options)
        : Node("panel_rectifier", options) {
        const std::string layout_yaml = declare_parameter<std::string>(
            "layout_yaml", "panel_layout.yaml"
        );
        board_frame_ = declare_parameter<std::string>(
            "board_frame", "aruco_board"
        );
        const std::string image_topic = declare_parameter<std::string>(
            "image_topic", "/d455_arm_wheel/color/image_raw"
        );
        const std::string camera_info_topic = declare_parameter<std::string>(
            "camera_info_topic", "/d455_arm_wheel/color/camera_info"
        );
        const std::string output_topic = declare_parameter<std::string>(
            "output_topic", "panel/image_rectified"
        );
        pixels_per_meter_ = declare_parameter<double>("pixels_per_meter", 1000.0);
        tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);
        use_latest_transform_ = declare_parameter<bool>(
            "use_latest_transform", true
        );

        load_layout(layout_yaml);
        left_border_m_ = declare_parameter<double>(
            "left_border_m", marker_size_
        );
        top_border_m_ = declare_parameter<double>(
            "top_border_m", marker_size_
        );
        draw_board_outline_ = declare_parameter<bool>(
            "draw_board_outline", true
        );
        outline_thickness_px_ = declare_parameter<int>(
            "outline_thickness_px", 2
        );

        if (!std::isfinite(pixels_per_meter_) || pixels_per_meter_ <= 0.0) {
            throw std::invalid_argument("pixels_per_meter must be finite and positive");
        }
        if (!std::isfinite(tf_timeout_s_) || tf_timeout_s_ < 0.0) {
            throw std::invalid_argument("tf_timeout_s must be finite and non-negative");
        }
        if (!std::isfinite(left_border_m_) || left_border_m_ < 0.0 ||
            !std::isfinite(top_border_m_) || top_border_m_ < 0.0) {
            throw std::invalid_argument("panel borders must be finite and non-negative");
        }
        if (outline_thickness_px_ <= 0) {
            throw std::invalid_argument("outline_thickness_px must be positive");
        }

        board_pixel_width_ = std::max(
            2, static_cast<int>(std::lround(board_width_ * pixels_per_meter_))
        );
        board_pixel_height_ = std::max(
            2, static_cast<int>(std::lround(board_height_ * pixels_per_meter_))
        );
        left_border_px_ = std::max(
            0, static_cast<int>(std::lround(left_border_m_ * pixels_per_meter_))
        );
        top_border_px_ = std::max(
            0, static_cast<int>(std::lround(top_border_m_ * pixels_per_meter_))
        );
        output_width_ = left_border_px_ + board_pixel_width_;
        output_height_ = top_border_px_ + board_pixel_height_; 

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto sensor_qos = rclcpp::SensorDataQoS();
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic,
            sensor_qos,
            std::bind(
                &PanelRectifier::on_camera_info, this, std::placeholders::_1
            )
        );
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic,
            sensor_qos,
            std::bind(&PanelRectifier::on_image, this, std::placeholders::_1)
        );
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            output_topic, sensor_qos
        );

        RCLCPP_INFO(
            get_logger(),
            "Panel rectifier: %.3f x %.3f m with left/top border %.3f/%.3f m -> %d x %d px at %.1f px/m",
            board_width_,
            board_height_,
            left_border_m_,
            top_border_m_,
            output_width_,
            output_height_,
            pixels_per_meter_
        );
    }

  private:
    struct RectificationData {
        cv::Mat map_x;
        cv::Mat map_y;
        cv::Mat camera_matrix;
        cv::Size image_size;
        std::array<double, 9> intrinsics;
        std::vector<double> distortion;
        std::string distortion_model;
        std::string camera_frame;
    };

    void load_layout(const std::string &layout_yaml) {
        try {
            const YAML::Node config = YAML::LoadFile(layout_yaml);
            board_width_ = config["board_width"].as<double>();
            board_height_ = config["board_height"].as<double>();
            marker_size_ = config["marker_size"].as<double>();
        } catch (const std::exception &error) {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to load panel layout '%s': %s",
                layout_yaml.c_str(),
                error.what()
            );
            throw;
        }

        if (!std::isfinite(board_width_) || board_width_ <= 0.0 ||
            !std::isfinite(board_height_) || board_height_ <= 0.0 ||
            !std::isfinite(marker_size_) || marker_size_ <= 0.0) {
            throw std::invalid_argument(
                "panel dimensions and marker_size must be finite and positive"
            );
        }
    }

    void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        if (message->width == 0 || message->height == 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring CameraInfo with zero image dimensions"
            );
            return;
        }

        {
            std::lock_guard<std::mutex> lock(rectification_mutex_);
            if (rectification_ &&
                rectification_->image_size.width ==
                    static_cast<int>(message->width) &&
                rectification_->image_size.height ==
                    static_cast<int>(message->height) &&
                rectification_->intrinsics == message->k &&
                rectification_->distortion == message->d &&
                rectification_->distortion_model == message->distortion_model &&
                rectification_->camera_frame == message->header.frame_id) {
                return;
            }
        }

        cv::Mat camera_matrix(3, 3, CV_64F);
        for (size_t index = 0; index < message->k.size(); ++index) {
            camera_matrix.at<double>(static_cast<int>(index / 3), static_cast<int>(index % 3)) =
                message->k[index];
        }
        if (camera_matrix.at<double>(0, 0) <= 0.0 ||
            camera_matrix.at<double>(1, 1) <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring CameraInfo with invalid focal lengths"
            );
            return;
        }

        cv::Mat distortion(
            1, static_cast<int>(message->d.size()), CV_64F, cv::Scalar(0.0)
        );
        for (size_t index = 0; index < message->d.size(); ++index) {
            distortion.at<double>(0, static_cast<int>(index)) = message->d[index];
        }

        RectificationData data;
        data.camera_matrix = camera_matrix;
        data.image_size = cv::Size(
            static_cast<int>(message->width), static_cast<int>(message->height)
        );
        data.intrinsics = message->k;
        data.distortion = message->d;
        data.distortion_model = message->distortion_model;
        data.camera_frame = message->header.frame_id;

        const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
        if (message->distortion_model == "equidistant") {
            cv::fisheye::initUndistortRectifyMap(
                camera_matrix,
                distortion,
                identity,
                camera_matrix,
                data.image_size,
                CV_32FC1,
                data.map_x,
                data.map_y
            );
        } else {
            cv::initUndistortRectifyMap(
                camera_matrix,
                distortion,
                identity,
                camera_matrix,
                data.image_size,
                CV_32FC1,
                data.map_x,
                data.map_y
            );
        }

        std::lock_guard<std::mutex> lock(rectification_mutex_);
        rectification_ = std::make_shared<RectificationData>(std::move(data));
    }

    std::array<cv::Point2f, 4> project_board_corners(
        const tf2::Transform &camera_to_board,
        const cv::Mat &camera_matrix
    ) const {
        const double half_width = board_width_ * 0.5;
        const double half_height = board_height_ * 0.5;
        const std::array<tf2::Vector3, 4> board_corners{
            tf2::Vector3(-half_width, half_height, 0.0),
            tf2::Vector3(half_width, half_height, 0.0),
            tf2::Vector3(half_width, -half_height, 0.0),
            tf2::Vector3(-half_width, -half_height, 0.0),
        };

        const double fx = camera_matrix.at<double>(0, 0);
        const double fy = camera_matrix.at<double>(1, 1);
        const double cx = camera_matrix.at<double>(0, 2);
        const double cy = camera_matrix.at<double>(1, 2);

        std::array<cv::Point2f, 4> image_corners;
        for (size_t index = 0; index < board_corners.size(); ++index) {
            const tf2::Vector3 camera_point = camera_to_board * board_corners[index];
            if (!std::isfinite(camera_point.x()) ||
                !std::isfinite(camera_point.y()) ||
                !std::isfinite(camera_point.z()) || camera_point.z() <= 1e-6) {
                throw std::runtime_error("panel corner is behind the camera");
            }
            image_corners[index] = cv::Point2f(
                static_cast<float>(fx * camera_point.x() / camera_point.z() + cx),
                static_cast<float>(fy * camera_point.y() / camera_point.z() + cy)
            );
        }
        return image_corners;
    }

    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::shared_ptr<const RectificationData> rectification;
        {
            std::lock_guard<std::mutex> lock(rectification_mutex_);
            rectification = rectification_;
        }
        if (!rectification) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for panel camera CameraInfo"
            );
            return;
        }
        if (static_cast<int>(message->width) != rectification->image_size.width ||
            static_cast<int>(message->height) != rectification->image_size.height) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Image dimensions %ux%u do not match CameraInfo dimensions %dx%d",
                message->width,
                message->height,
                rectification->image_size.width,
                rectification->image_size.height
            );
            return;
        }

        const std::string camera_frame = message->header.frame_id.empty()
                                             ? rectification->camera_frame
                                             : message->header.frame_id;
        if (camera_frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Camera image has no frame_id"
            );
            return;
        }

        tf2::Transform camera_to_board;
        try {
            const rclcpp::Time transform_time = use_latest_transform_
                                                    ? rclcpp::Time(
                                                          0,
                                                          0,
                                                          get_clock()->get_clock_type()
                                                      )
                                                    : rclcpp::Time(message->header.stamp);
            const auto transform = tf_buffer_->lookupTransform(
                camera_frame,
                board_frame_,
                transform_time,
                rclcpp::Duration::from_seconds(tf_timeout_s_)
            );
            tf2::fromMsg(transform.transform, camera_to_board);
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Panel rectifier TF lookup %s <- %s failed: %s",
                camera_frame.c_str(),
                board_frame_.c_str(),
                error.what()
            );
            return;
        }

        try {
            const cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(
                message, sensor_msgs::image_encodings::BGR8
            );
            cv::Mat undistorted;
            cv::remap(
                cv_image->image,
                undistorted,
                rectification->map_x,
                rectification->map_y,
                cv::INTER_LINEAR
            );

            const auto source_corners = project_board_corners(
                camera_to_board, rectification->camera_matrix
            );
            const float board_left = static_cast<float>(left_border_px_);
            const float board_top = static_cast<float>(top_border_px_);
            const float board_right = static_cast<float>(
                left_border_px_ + board_pixel_width_ - 1
            );
            const float board_bottom = static_cast<float>(
                top_border_px_ + board_pixel_height_ - 1
            );
            const std::array<cv::Point2f, 4> destination_corners{
                cv::Point2f(board_left, board_top),
                cv::Point2f(board_right, board_top),
                cv::Point2f(board_right, board_bottom),
                cv::Point2f(board_left, board_bottom),
            };

            const cv::Mat homography = cv::getPerspectiveTransform(
                source_corners.data(), destination_corners.data()
            );
            cv::Mat panel_image;
            cv::warpPerspective(
                undistorted,
                panel_image,
                homography,
                cv::Size(output_width_, output_height_),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT
            );
            if (draw_board_outline_) {
                cv::rectangle(
                    panel_image,
                    cv::Point(left_border_px_, top_border_px_),
                    cv::Point(
                        left_border_px_ + board_pixel_width_ - 1,
                        top_border_px_ + board_pixel_height_ - 1
                    ),
                    cv::Scalar(0, 255, 0),
                    outline_thickness_px_,
                    cv::LINE_AA
                );
            }

            std_msgs::msg::Header output_header = message->header;
            output_header.frame_id = board_frame_;
            image_pub_->publish(
                *cv_bridge::CvImage(
                     output_header,
                     sensor_msgs::image_encodings::BGR8,
                     panel_image
                 )
                     .toImageMsg()
            );
        } catch (const cv_bridge::Exception &error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Panel image conversion failed: %s",
                error.what()
            );
        } catch (const cv::Exception &error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Panel image rectification failed: %s",
                error.what()
            );
        } catch (const std::exception &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot rectify panel image: %s",
                error.what()
            );
        }
    }

    std::string board_frame_;
    double board_width_{0.0};
    double board_height_{0.0};
    double marker_size_{0.0};
    double pixels_per_meter_{1000.0};
    double tf_timeout_s_{0.1};
    double left_border_m_{0.0};
    double top_border_m_{0.0};
    bool use_latest_transform_{true};
    bool draw_board_outline_{true};
    int outline_thickness_px_{2};
    int board_pixel_width_{0};
    int board_pixel_height_{0};
    int left_border_px_{0};
    int top_border_px_{0};
    int output_width_{0};
    int output_height_{0};

    std::mutex rectification_mutex_;
    std::shared_ptr<RectificationData> rectification_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelRectifier)
