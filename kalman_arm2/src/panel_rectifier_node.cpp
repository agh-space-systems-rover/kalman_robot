#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
        const std::string depth_topic = declare_parameter<std::string>(
            "depth_topic", "/d455_arm_wheel/depth/image_raw"
        );
        const std::string camera_info_topic = declare_parameter<std::string>(
            "camera_info_topic", "/d455_arm_wheel/color/camera_info"
        );
        const std::string output_topic = declare_parameter<std::string>(
            "output_topic", "panel/image_rectified"
        );
        const std::string height_topic = declare_parameter<std::string>(
            "height_topic", "panel/height"
        );
        const std::string valid_topic = declare_parameter<std::string>(
            "valid_topic", "panel/valid"
        );
        const std::string pixel_transform_topic = declare_parameter<std::string>(
            "pixel_transform_topic", "panel/pixel_to_panel"
        );
        pixels_per_meter_ = declare_parameter<double>("pixels_per_meter", 1000.0);
        tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);
        use_latest_transform_ = declare_parameter<bool>(
            "use_latest_transform", true
        );
        min_panel_height_m_ = declare_parameter<double>(
            "min_panel_height_m", -0.03
        );
        max_panel_height_m_ = declare_parameter<double>(
            "max_panel_height_m", 0.25
        );
        depth_edge_threshold_m_ = declare_parameter<double>(
            "depth_edge_threshold_m", 0.02
        );
        max_splat_radius_px_ = declare_parameter<int>(
            "max_splat_radius_px", 3
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

        validate_parameters();
        configure_output_geometry();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS input_qos(1);
        input_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        input_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        image_sub_.subscribe(this, image_topic, input_qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, depth_topic, input_qos.get_rmw_qos_profile());
        camera_info_sub_.subscribe(
            this, camera_info_topic, input_qos.get_rmw_qos_profile()
        );

        synchronizer_ = std::make_shared<Synchronizer>(
            image_sub_, depth_sub_, camera_info_sub_, 10
        );
        synchronizer_->registerCallback(std::bind(
            &PanelRectifier::on_rgbd,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        ));

        const auto output_qos = rclcpp::SensorDataQoS();
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            output_topic, output_qos
        );
        height_pub_ = create_publisher<sensor_msgs::msg::Image>(
            height_topic, output_qos
        );
        valid_pub_ = create_publisher<sensor_msgs::msg::Image>(
            valid_topic, output_qos
        );
        auto transform_qos = rclcpp::QoS(1).reliable().transient_local();
        pixel_transform_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                pixel_transform_topic, transform_qos
            );
        publish_pixel_transform();

        RCLCPP_INFO(
            get_logger(),
            "Orthographic panel renderer: %.3f x %.3f m with left/top border "
            "%.3f/%.3f m -> %d x %d px at %.1f px/m",
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
    using Synchronizer = message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;

    struct CalibrationData {
        cv::Mat map_x;
        cv::Mat map_y;
        cv::Mat camera_matrix;
        cv::Size image_size;
        std::array<double, 9> intrinsics;
        std::vector<double> distortion;
        std::string distortion_model;
        std::vector<float> ray_x;
        std::vector<float> ray_y;
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
    }

    void validate_parameters() const {
        if (!std::isfinite(board_width_) || board_width_ <= 0.0 ||
            !std::isfinite(board_height_) || board_height_ <= 0.0 ||
            !std::isfinite(marker_size_) || marker_size_ <= 0.0) {
            throw std::invalid_argument(
                "panel dimensions and marker_size must be finite and positive"
            );
        }
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
        if (!std::isfinite(min_panel_height_m_) ||
            !std::isfinite(max_panel_height_m_) ||
            min_panel_height_m_ >= max_panel_height_m_) {
            throw std::invalid_argument("invalid panel height range");
        }
        if (!std::isfinite(depth_edge_threshold_m_) ||
            depth_edge_threshold_m_ <= 0.0) {
            throw std::invalid_argument(
                "depth_edge_threshold_m must be finite and positive"
            );
        }
        if (max_splat_radius_px_ < 0 || outline_thickness_px_ <= 0) {
            throw std::invalid_argument("invalid rasterization pixel sizes");
        }
    }

    void configure_output_geometry() {
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
    }

    std::shared_ptr<const CalibrationData> calibration_for(
        const sensor_msgs::msg::CameraInfo &message
    ) {
        if (message.width == 0 || message.height == 0) {
            throw std::runtime_error("CameraInfo has zero image dimensions");
        }
        if (calibration_ &&
            calibration_->image_size.width == static_cast<int>(message.width) &&
            calibration_->image_size.height == static_cast<int>(message.height) &&
            calibration_->intrinsics == message.k &&
            calibration_->distortion == message.d &&
            calibration_->distortion_model == message.distortion_model) {
            return calibration_;
        }

        auto data = std::make_shared<CalibrationData>();
        data->camera_matrix = cv::Mat(3, 3, CV_64F);
        for (size_t index = 0; index < message.k.size(); ++index) {
            data->camera_matrix.at<double>(
                static_cast<int>(index / 3), static_cast<int>(index % 3)
            ) = message.k[index];
        }
        const double focal_x = data->camera_matrix.at<double>(0, 0);
        const double focal_y = data->camera_matrix.at<double>(1, 1);
        if (focal_x <= 0.0 || focal_y <= 0.0) {
            throw std::runtime_error("CameraInfo has invalid focal lengths");
        }

        cv::Mat distortion(
            1, static_cast<int>(message.d.size()), CV_64F, cv::Scalar(0.0)
        );
        for (size_t index = 0; index < message.d.size(); ++index) {
            distortion.at<double>(0, static_cast<int>(index)) = message.d[index];
        }

        data->image_size = cv::Size(
            static_cast<int>(message.width), static_cast<int>(message.height)
        );
        data->intrinsics = message.k;
        data->distortion = message.d;
        data->distortion_model = message.distortion_model;

        const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
        if (message.distortion_model == "equidistant") {
            cv::fisheye::initUndistortRectifyMap(
                data->camera_matrix,
                distortion,
                identity,
                data->camera_matrix,
                data->image_size,
                CV_32FC1,
                data->map_x,
                data->map_y
            );
        } else {
            cv::initUndistortRectifyMap(
                data->camera_matrix,
                distortion,
                identity,
                data->camera_matrix,
                data->image_size,
                CV_32FC1,
                data->map_x,
                data->map_y
            );
        }

        const double center_x = data->camera_matrix.at<double>(0, 2);
        const double center_y = data->camera_matrix.at<double>(1, 2);
        data->ray_x.resize(message.width);
        data->ray_y.resize(message.height);
        for (size_t column = 0; column < message.width; ++column) {
            data->ray_x[column] = static_cast<float>(
                (static_cast<double>(column) - center_x) / focal_x
            );
        }
        for (size_t row = 0; row < message.height; ++row) {
            data->ray_y[row] = static_cast<float>(
                (static_cast<double>(row) - center_y) / focal_y
            );
        }

        calibration_ = data;
        return data;
    }

    tf2::Transform board_from_camera(
        const std::string &camera_frame, const builtin_interfaces::msg::Time &stamp
    ) const {
        const rclcpp::Time transform_time = use_latest_transform_
                                                ? rclcpp::Time(
                                                      0,
                                                      0,
                                                      get_clock()->get_clock_type()
                                                  )
                                                : rclcpp::Time(stamp);
        const auto transform = tf_buffer_->lookupTransform(
            board_frame_,
            camera_frame,
            transform_time,
            rclcpp::Duration::from_seconds(tf_timeout_s_)
        );
        tf2::Transform result;
        tf2::fromMsg(transform.transform, result);
        return result;
    }

    cv::Mat depth_in_meters(
        const sensor_msgs::msg::Image::ConstSharedPtr &message
    ) const {
        const cv_bridge::CvImageConstPtr depth = cv_bridge::toCvShare(message);
        cv::Mat meters;
        if (message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            message->encoding == sensor_msgs::image_encodings::MONO16) {
            depth->image.convertTo(meters, CV_32FC1, 0.001);
        } else if (
            message->encoding == sensor_msgs::image_encodings::TYPE_32FC1
        ) {
            meters = depth->image;
        } else {
            throw std::runtime_error(
                "unsupported depth encoding '" + message->encoding + "'"
            );
        }
        return meters;
    }

    static bool valid_depth(float depth) {
        return std::isfinite(depth) && depth > 0.0F;
    }

    bool is_depth_edge(const cv::Mat &depth, int row, int column, float value) const {
        constexpr std::array<std::array<int, 2>, 4> offsets{
            std::array<int, 2>{-1, 0},
            std::array<int, 2>{1, 0},
            std::array<int, 2>{0, -1},
            std::array<int, 2>{0, 1},
        };
        for (const auto &offset : offsets) {
            const int neighbor_row = row + offset[0];
            const int neighbor_column = column + offset[1];
            if (neighbor_row < 0 || neighbor_row >= depth.rows ||
                neighbor_column < 0 || neighbor_column >= depth.cols) {
                return true;
            }
            const float neighbor = depth.at<float>(neighbor_row, neighbor_column);
            if (!valid_depth(neighbor) ||
                std::abs(neighbor - value) > depth_edge_threshold_m_) {
                return true;
            }
        }
        return false;
    }

    void fill_unknown_background(cv::Mat &image) const {
        constexpr int tile_size = 12;
        for (int row = 0; row < image.rows; ++row) {
            cv::Vec3b *pixels = image.ptr<cv::Vec3b>(row);
            for (int column = 0; column < image.cols; ++column) {
                const bool alternate =
                    ((row / tile_size) + (column / tile_size)) % 2 != 0;
                const unsigned char intensity = alternate ? 24 : 40;
                pixels[column] = cv::Vec3b(intensity, intensity, intensity);
            }
        }
    }

    void render_points(
        const cv::Mat &color,
        const cv::Mat &depth,
        const CalibrationData &calibration,
        const tf2::Transform &board_from_camera,
        cv::Mat &output,
        cv::Mat &height,
        cv::Mat &valid
    ) const {
        const double minimum_x = -0.5 * board_width_ - left_border_m_;
        const double maximum_x = 0.5 * board_width_;
        const double minimum_y = -0.5 * board_height_;
        const double maximum_y = 0.5 * board_height_ + top_border_m_;
        const double average_focal_length = 0.5 * (
            calibration.camera_matrix.at<double>(0, 0) +
            calibration.camera_matrix.at<double>(1, 1)
        );

        for (int row = 0; row < depth.rows; ++row) {
            const float *depth_pixels = depth.ptr<float>(row);
            const cv::Vec3b *color_pixels = color.ptr<cv::Vec3b>(row);
            for (int column = 0; column < depth.cols; ++column) {
                const float camera_depth = depth_pixels[column];
                if (!valid_depth(camera_depth)) {
                    continue;
                }

                const tf2::Vector3 camera_point(
                    calibration.ray_x[static_cast<size_t>(column)] * camera_depth,
                    calibration.ray_y[static_cast<size_t>(row)] * camera_depth,
                    camera_depth
                );
                const tf2::Vector3 panel_point =
                    board_from_camera * camera_point;
                if (panel_point.x() < minimum_x || panel_point.x() > maximum_x ||
                    panel_point.y() < minimum_y || panel_point.y() > maximum_y ||
                    panel_point.z() < min_panel_height_m_ ||
                    panel_point.z() > max_panel_height_m_) {
                    continue;
                }

                const int output_x = static_cast<int>(std::lround(
                    left_border_px_ +
                    (panel_point.x() + 0.5 * board_width_) * pixels_per_meter_
                ));
                const int output_y = static_cast<int>(std::lround(
                    top_border_px_ +
                    (0.5 * board_height_ - panel_point.y()) * pixels_per_meter_
                ));
                if (output_x < 0 || output_x >= output_width_ || output_y < 0 ||
                    output_y >= output_height_) {
                    continue;
                }

                int radius = static_cast<int>(std::ceil(
                    0.5 * pixels_per_meter_ * camera_depth /
                    average_focal_length
                ));
                radius = std::clamp(radius, 0, max_splat_radius_px_);
                if (is_depth_edge(depth, row, column, camera_depth)) {
                    radius = 0;
                }

                for (int target_y = std::max(0, output_y - radius);
                     target_y <= std::min(output_height_ - 1, output_y + radius);
                     ++target_y) {
                    float *height_pixels = height.ptr<float>(target_y);
                    unsigned char *valid_pixels = valid.ptr<unsigned char>(target_y);
                    cv::Vec3b *output_pixels = output.ptr<cv::Vec3b>(target_y);
                    for (int target_x = std::max(0, output_x - radius);
                         target_x <= std::min(output_width_ - 1, output_x + radius);
                         ++target_x) {
                        if (valid_pixels[target_x] == 0 ||
                            panel_point.z() > height_pixels[target_x]) {
                            height_pixels[target_x] = static_cast<float>(
                                panel_point.z()
                            );
                            valid_pixels[target_x] = 255;
                            output_pixels[target_x] = color_pixels[column];
                        }
                    }
                }
            }
        }
    }

    void publish_pixel_transform() {
        std_msgs::msg::Float64MultiArray transform;
        transform.layout.dim.resize(2);
        transform.layout.dim[0].label = "rows";
        transform.layout.dim[0].size = 3;
        transform.layout.dim[0].stride = 9;
        transform.layout.dim[1].label = "columns";
        transform.layout.dim[1].size = 3;
        transform.layout.dim[1].stride = 3;

        const double meters_per_pixel = 1.0 / pixels_per_meter_;
        transform.data = {
            meters_per_pixel,
            0.0,
            -static_cast<double>(left_border_px_) * meters_per_pixel -
                0.5 * board_width_,
            0.0,
            -meters_per_pixel,
            static_cast<double>(top_border_px_) * meters_per_pixel +
                0.5 * board_height_,
            0.0,
            0.0,
            1.0,
        };
        pixel_transform_pub_->publish(transform);
    }

    void publish_outputs(
        const std_msgs::msg::Header &input_header,
        const cv::Mat &image,
        const cv::Mat &height,
        const cv::Mat &valid
    ) {
        std_msgs::msg::Header output_header = input_header;
        output_header.frame_id = board_frame_;
        image_pub_->publish(
            *cv_bridge::CvImage(
                 output_header, sensor_msgs::image_encodings::BGR8, image
             )
                 .toImageMsg()
        );
        height_pub_->publish(
            *cv_bridge::CvImage(
                 output_header, sensor_msgs::image_encodings::TYPE_32FC1, height
             )
                 .toImageMsg()
        );
        valid_pub_->publish(
            *cv_bridge::CvImage(
                 output_header, sensor_msgs::image_encodings::MONO8, valid
             )
                 .toImageMsg()
        );
    }

    void on_rgbd(
        const sensor_msgs::msg::Image::ConstSharedPtr &image_message,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_message,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_message
    ) {
        try {
            if (image_message->width != depth_message->width ||
                image_message->height != depth_message->height ||
                image_message->width != camera_info_message->width ||
                image_message->height != camera_info_message->height) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Aligned RGB-D dimensions differ: color=%ux%u depth=%ux%u "
                    "info=%ux%u",
                    image_message->width,
                    image_message->height,
                    depth_message->width,
                    depth_message->height,
                    camera_info_message->width,
                    camera_info_message->height
                );
                return;
            }

            const auto calibration = calibration_for(*camera_info_message);
            const cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(
                image_message, sensor_msgs::image_encodings::BGR8
            );
            const cv::Mat depth_meters = depth_in_meters(depth_message);

            cv::Mat rectified_color;
            cv::Mat rectified_depth;
            cv::remap(
                color->image,
                rectified_color,
                calibration->map_x,
                calibration->map_y,
                cv::INTER_LINEAR
            );
            cv::remap(
                depth_meters,
                rectified_depth,
                calibration->map_x,
                calibration->map_y,
                cv::INTER_NEAREST,
                cv::BORDER_CONSTANT,
                cv::Scalar(0.0)
            );

            const std::string camera_frame = image_message->header.frame_id.empty()
                                                 ? camera_info_message->header.frame_id
                                                 : image_message->header.frame_id;
            if (camera_frame.empty()) {
                throw std::runtime_error("camera frame_id is empty");
            }
            const tf2::Transform board_from_camera_transform = board_from_camera(
                camera_frame, image_message->header.stamp
            );

            cv::Mat output(
                output_height_, output_width_, CV_8UC3
            );
            fill_unknown_background(output);
            cv::Mat height(
                output_height_,
                output_width_,
                CV_32FC1,
                cv::Scalar(std::numeric_limits<float>::quiet_NaN())
            );
            cv::Mat valid(
                output_height_, output_width_, CV_8UC1, cv::Scalar(0)
            );
            render_points(
                rectified_color,
                rectified_depth,
                *calibration,
                board_from_camera_transform,
                output,
                height,
                valid
            );

            if (draw_board_outline_) {
                cv::rectangle(
                    output,
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
            publish_outputs(image_message->header, output, height, valid);
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Orthographic panel TF lookup failed: %s",
                error.what()
            );
        } catch (const cv_bridge::Exception &error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "RGB-D conversion failed: %s",
                error.what()
            );
        } catch (const cv::Exception &error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Orthographic rendering failed: %s",
                error.what()
            );
        } catch (const std::exception &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cannot render orthographic panel: %s",
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
    double min_panel_height_m_{-0.03};
    double max_panel_height_m_{0.25};
    double depth_edge_threshold_m_{0.02};
    bool use_latest_transform_{true};
    bool draw_board_outline_{true};
    int max_splat_radius_px_{3};
    int outline_thickness_px_{2};
    int board_pixel_width_{0};
    int board_pixel_height_{0};
    int left_border_px_{0};
    int top_border_px_{0};
    int output_width_{0};
    int output_height_{0};

    std::shared_ptr<CalibrationData> calibration_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
    std::shared_ptr<Synchronizer> synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr height_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr valid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pixel_transform_pub_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelRectifier)
