#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <kalman_interfaces/msg/instance_contour_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
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
            "pixel_transform_topic", "panel/homography"
        );
        const std::string contour_topic = declare_parameter<std::string>(
            "contour_topic", "/d455_arm_wheel/yolo_contours"
        );
        const std::string segmentation_debug_topic =
            declare_parameter<std::string>(
                "segmentation_debug_topic", "panel/segmentation_debug"
            );
        const std::string rectified_detections_topic =
            declare_parameter<std::string>(
                "rectified_detections_topic", "panel/detections_rectified"
            );
        projection_cache_size_ = declare_parameter<int>(
            "projection_cache_size", 30
        );
        sync_queue_size_ = declare_parameter<int>("sync_queue_size", 30);
        sync_tolerance_s_ = declare_parameter<double>("sync_tolerance_s", 0.02);
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

        rclcpp::QoS input_qos(5);
        input_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        input_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        image_sub_.subscribe(this, image_topic, input_qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, depth_topic, input_qos.get_rmw_qos_profile());
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic,
            input_qos,
            std::bind(
                &PanelRectifier::on_camera_info, this, std::placeholders::_1
            )
        );

        synchronizer_ = std::make_shared<Synchronizer>(
            SynchronizationPolicy(sync_queue_size_), image_sub_, depth_sub_
        );
        synchronizer_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(sync_tolerance_s_)
        );
        synchronizer_->registerCallback(std::bind(
            &PanelRectifier::on_rgbd,
            this,
            std::placeholders::_1,
            std::placeholders::_2
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
        segmentation_debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
            segmentation_debug_topic, output_qos
        );
        rectified_detections_pub_ =
            create_publisher<vision_msgs::msg::Detection2DArray>(
                rectified_detections_topic, rclcpp::QoS(10).reliable()
            );
        contour_sub_ = create_subscription<
            kalman_interfaces::msg::InstanceContourArray>(
            contour_topic,
            output_qos,
            std::bind(
                &PanelRectifier::on_contours, this, std::placeholders::_1
            )
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
            "%.3f/%.3f m -> %d x %d px at %.1f px/m; approximate RGB-D sync "
            "queue=%d tolerance=%.3f s",
            board_width_,
            board_height_,
            left_border_m_,
            top_border_m_,
            output_width_,
            output_height_,
            pixels_per_meter_,
            sync_queue_size_,
            sync_tolerance_s_
        );
    }

  private:
    using SynchronizationPolicy =
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SynchronizationPolicy>;

    struct ProjectionCacheEntry {
        builtin_interfaces::msg::Time stamp;
        cv::Mat image;
        cv::Mat source_indices;
        int source_width{0};
        int source_height{0};
    };

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
        if (projection_cache_size_ <= 0) {
            throw std::invalid_argument("projection_cache_size must be positive");
        }
        if (sync_queue_size_ <= 0) {
            throw std::invalid_argument("sync_queue_size must be positive");
        }
        if (!std::isfinite(sync_tolerance_s_) || sync_tolerance_s_ <= 0.0) {
            throw std::invalid_argument(
                "sync_tolerance_s must be finite and positive"
            );
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
        cv::Mat &valid,
        cv::Mat &source_indices
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

                const float mapped_x = calibration.map_x.at<float>(row, column);
                const float mapped_y = calibration.map_y.at<float>(row, column);
                int raw_index = -1;
                if (std::isfinite(mapped_x) && std::isfinite(mapped_y) &&
                    mapped_x > -0.5F &&
                    mapped_x < static_cast<float>(calibration.image_size.width) -
                                   0.5F &&
                    mapped_y > -0.5F &&
                    mapped_y < static_cast<float>(calibration.image_size.height) -
                                   0.5F) {
                    const int raw_x = static_cast<int>(std::lround(mapped_x));
                    const int raw_y = static_cast<int>(std::lround(mapped_y));
                    raw_index = raw_y * calibration.image_size.width + raw_x;
                }

                for (int target_y = std::max(0, output_y - radius);
                     target_y <= std::min(output_height_ - 1, output_y + radius);
                     ++target_y) {
                    float *height_pixels = height.ptr<float>(target_y);
                    unsigned char *valid_pixels = valid.ptr<unsigned char>(target_y);
                    cv::Vec3b *output_pixels = output.ptr<cv::Vec3b>(target_y);
                    int *source_index_pixels = source_indices.ptr<int>(target_y);
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
                            source_index_pixels[target_x] = raw_index;
                        }
                    }
                }
            }
        }
    }

    static bool same_stamp(
        const builtin_interfaces::msg::Time &left,
        const builtin_interfaces::msg::Time &right
    ) {
        return left.sec == right.sec && left.nanosec == right.nanosec;
    }

    static cv::Vec3b color_for_class(const std::string &class_id) {
        uint32_t hash = 2166136261U;
        for (const unsigned char byte : class_id) {
            hash ^= byte;
            hash *= 16777619U;
        }

        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hash % 180U, 220, 255));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        return bgr.at<cv::Vec3b>(0, 0);
    }

    void cache_projection(
        const builtin_interfaces::msg::Time &stamp,
        const cv::Mat &image,
        const cv::Mat &source_indices,
        int source_width,
        int source_height
    ) {
        ProjectionCacheEntry entry{
            stamp,
            image.clone(),
            source_indices.clone(),
            source_width,
            source_height,
        };
        std::lock_guard<std::mutex> lock(projection_cache_mutex_);
        projection_cache_.push_back(std::move(entry));
        while (projection_cache_.size() >
               static_cast<size_t>(projection_cache_size_)) {
            projection_cache_.pop_front();
        }
    }

    void on_contours(
        const kalman_interfaces::msg::InstanceContourArray::ConstSharedPtr message
    ) {
        ProjectionCacheEntry cached;
        {
            std::lock_guard<std::mutex> lock(projection_cache_mutex_);
            const auto entry = std::find_if(
                projection_cache_.begin(),
                projection_cache_.end(),
                [&message](const ProjectionCacheEntry &candidate) {
                    return same_stamp(candidate.stamp, message->header.stamp);
                }
            );
            if (entry == projection_cache_.end()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "No orthographic projection cached for contour stamp %d.%09u",
                    message->header.stamp.sec,
                    message->header.stamp.nanosec
                );
                return;
            }
            cached = *entry;
        }

        if (message->image_width != static_cast<uint32_t>(cached.source_width) ||
            message->image_height != static_cast<uint32_t>(cached.source_height)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Contour dimensions %ux%u differ from cached source %dx%d",
                message->image_width,
                message->image_height,
                cached.source_width,
                cached.source_height
            );
            return;
        }

        cv::Mat labels(
            cached.source_height,
            cached.source_width,
            CV_16UC1,
            cv::Scalar(0)
        );
        std::vector<cv::Vec3b> label_colors(message->instances.size() + 1);
        const size_t maximum_labels = std::min<size_t>(
            message->instances.size(), std::numeric_limits<uint16_t>::max()
        );
        for (size_t index = 0; index < maximum_labels; ++index) {
            const auto &instance = message->instances[index];
            if (instance.points.size() < 3) {
                continue;
            }
            std::vector<cv::Point> contour;
            contour.reserve(instance.points.size());
            for (const auto &point : instance.points) {
                contour.emplace_back(point.x, point.y);
            }
            const uint16_t label = static_cast<uint16_t>(index + 1);
            const std::vector<std::vector<cv::Point>> contours{std::move(contour)};
            cv::fillPoly(labels, contours, cv::Scalar(label));
            label_colors[label] = color_for_class(instance.class_id);
        }

        cv::Mat debug = cached.image.clone();
        std::vector<int> minimum_x(message->instances.size() + 1, debug.cols);
        std::vector<int> minimum_y(message->instances.size() + 1, debug.rows);
        std::vector<int> maximum_x(message->instances.size() + 1, -1);
        std::vector<int> maximum_y(message->instances.size() + 1, -1);
        constexpr unsigned int overlay_weight = 128;
        for (int row = 0; row < debug.rows; ++row) {
            cv::Vec3b *debug_pixels = debug.ptr<cv::Vec3b>(row);
            const int *source_pixels = cached.source_indices.ptr<int>(row);
            for (int column = 0; column < debug.cols; ++column) {
                const int source_index = source_pixels[column];
                if (source_index < 0) {
                    continue;
                }
                const uint16_t label = labels.ptr<uint16_t>()[source_index];
                if (label == 0 || label >= label_colors.size()) {
                    continue;
                }
                minimum_x[label] = std::min(minimum_x[label], column);
                minimum_y[label] = std::min(minimum_y[label], row);
                maximum_x[label] = std::max(maximum_x[label], column);
                maximum_y[label] = std::max(maximum_y[label], row);

                const cv::Vec3b color = label_colors[label];
                for (int channel = 0; channel < 3; ++channel) {
                    debug_pixels[column][channel] = static_cast<unsigned char>(
                        (static_cast<unsigned int>(debug_pixels[column][channel]) *
                             (256U - overlay_weight) +
                         static_cast<unsigned int>(color[channel]) * overlay_weight) /
                        256U
                    );
                }
            }
        }

        std_msgs::msg::Header header = message->header;
        header.frame_id = board_frame_;

        vision_msgs::msg::Detection2DArray detections;
        detections.header = header;
        for (size_t index = 0; index < maximum_labels; ++index) {
            const size_t label = index + 1;
            if (maximum_x[label] < minimum_x[label] ||
                maximum_y[label] < minimum_y[label]) {
                continue;
            }

            const auto &instance = message->instances[index];
            vision_msgs::msg::Detection2D detection;
            detection.id = instance.id;
            detection.bbox.center.position.x =
                0.5 * static_cast<double>(minimum_x[label] + maximum_x[label]);
            detection.bbox.center.position.y =
                0.5 * static_cast<double>(minimum_y[label] + maximum_y[label]);
            detection.bbox.size_x =
                static_cast<double>(maximum_x[label] - minimum_x[label] + 1);
            detection.bbox.size_y =
                static_cast<double>(maximum_y[label] - minimum_y[label] + 1);
            detection.results.resize(1);
            detection.results[0].hypothesis.class_id = instance.class_id;
            detection.results[0].hypothesis.score = instance.score;
            detections.detections.push_back(std::move(detection));
        }
        rectified_detections_pub_->publish(detections);

        segmentation_debug_pub_->publish(
            *cv_bridge::CvImage(
                 header, sensor_msgs::image_encodings::BGR8, debug
             )
                 .toImageMsg()
        );
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

        // Transient-local data is not replayed to volatile rosbridge subscribers.
        // Publish alongside every image so late ground-station clients receive it.
        publish_pixel_transform();
    }

    void on_camera_info(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr message
    ) {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        latest_camera_info_ = message;
    }

    void on_rgbd(
        const sensor_msgs::msg::Image::ConstSharedPtr &image_message,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_message
    ) {
        try {
            sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_message;
            {
                std::lock_guard<std::mutex> lock(camera_info_mutex_);
                camera_info_message = latest_camera_info_;
            }
            if (!camera_info_message) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Cannot render orthographic panel before camera info arrives"
                );
                return;
            }

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
            cv::Mat source_indices(
                output_height_, output_width_, CV_32SC1, cv::Scalar(-1)
            );
            render_points(
                rectified_color,
                rectified_depth,
                *calibration,
                board_from_camera_transform,
                output,
                height,
                valid,
                source_indices
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
            cache_projection(
                image_message->header.stamp,
                output,
                source_indices,
                static_cast<int>(image_message->width),
                static_cast<int>(image_message->height)
            );
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
    int projection_cache_size_{30};
    int sync_queue_size_{30};
    double sync_tolerance_s_{0.02};

    std::shared_ptr<CalibrationData> calibration_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<Synchronizer> synchronizer_;

    std::mutex camera_info_mutex_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
    rclcpp::Subscription<kalman_interfaces::msg::InstanceContourArray>::SharedPtr
        contour_sub_;

    std::mutex projection_cache_mutex_;
    std::deque<ProjectionCacheEntry> projection_cache_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr height_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr valid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segmentation_debug_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
        rectified_detections_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        pixel_transform_pub_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelRectifier)
