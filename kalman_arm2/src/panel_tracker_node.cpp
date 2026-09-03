#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <kalman_interfaces/action/calibrate_panel_marker_ids.hpp>
#include <kalman_interfaces/srv/set_panel_marker_ids.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace kalman_arm2 {

class PanelTracker : public rclcpp::Node {
  public:
    using CalibrateAction =
        kalman_interfaces::action::CalibratePanelMarkerIds;
    using CalibrationGoalHandle =
        rclcpp_action::ServerGoalHandle<CalibrateAction>;
    using SetMarkerIds = kalman_interfaces::srv::SetPanelMarkerIds;

    PanelTracker(const rclcpp::NodeOptions &options)
        : Node("panel_tracker", options) {
        declare_parameter<std::string>("tracking_frame", "odom");
        layout_yaml_path_ =
            declare_parameter<std::string>("layout_yaml", "panel_layout.yaml");
        declare_parameter<std::string>("board_frame", "aruco_board");
        declare_parameter<std::string>(
            "detection_topic", "/d455_arm/aruco_detections"
        );
        declare_parameter<std::string>(
            "depth_topic", "/d455_arm_wheel/depth/image_raw"
        );
        declare_parameter<std::string>(
            "camera_info_topic", "/d455_arm_wheel/color/camera_info"
        );
        declare_parameter<double>("ema_alpha", 0.2);
        declare_parameter<int>("ee_marker_id", 31);
        declare_parameter<std::string>("ee_marker_frame", "aruco_under_j6");
        declare_parameter<int>("secondary_ee_marker_id", 30);
        declare_parameter<std::string>(
            "secondary_ee_marker_frame", "aruco_left_of_j6"
        );
        declare_parameter<std::string>("ee_frame", "arm_link_gripper");
        declare_parameter<int>("calibration_required_confirmations", 5);
        declare_parameter<double>("calibration_timeout_seconds", 10.0);
        declare_parameter<double>("calibration_max_distance_error", 0.05);
        declare_parameter<bool>("depth_refinement_enabled", true);
        declare_parameter<double>("depth_max_age_s", 0.1);
        declare_parameter<double>("plane_roi_margin_m", 0.02);
        declare_parameter<double>("plane_initial_distance_m", 0.02);
        declare_parameter<double>("plane_min_residual_threshold_m", 0.002);
        declare_parameter<double>("plane_robust_sigma_multiplier", 3.0);
        declare_parameter<int>("plane_iterations", 3);
        declare_parameter<double>("plane_max_normal_update_deg", 5.0);
        declare_parameter<double>("plane_max_offset_update_m", 0.01);
        declare_parameter<int>("plane_min_points", 200);
        declare_parameter<int>("plane_pixel_stride", 2);

        get_parameter("tracking_frame", tracking_frame_);
        get_parameter("board_frame", board_frame_);
        get_parameter("detection_topic", detection_topic_);
        get_parameter("depth_topic", depth_topic_);
        get_parameter("camera_info_topic", camera_info_topic_);
        get_parameter("ema_alpha", ema_alpha_);
        get_parameter("ee_frame", ee_frame_);
        get_parameter(
            "calibration_required_confirmations",
            default_calibration_confirmations_
        );
        get_parameter(
            "calibration_timeout_seconds", default_calibration_timeout_seconds_
        );
        get_parameter(
            "calibration_max_distance_error", calibration_max_distance_error_
        );
        get_parameter("depth_refinement_enabled", depth_refinement_enabled_);
        get_parameter("depth_max_age_s", depth_max_age_s_);
        get_parameter("plane_roi_margin_m", plane_roi_margin_m_);
        get_parameter("plane_initial_distance_m", plane_initial_distance_m_);
        get_parameter(
            "plane_min_residual_threshold_m", plane_min_residual_threshold_m_
        );
        get_parameter(
            "plane_robust_sigma_multiplier", plane_robust_sigma_multiplier_
        );
        get_parameter("plane_iterations", plane_iterations_);
        double plane_max_normal_update_deg = 5.0;
        get_parameter(
            "plane_max_normal_update_deg", plane_max_normal_update_deg
        );
        plane_max_normal_update_rad_ =
            plane_max_normal_update_deg * M_PI / 180.0;
        get_parameter("plane_max_offset_update_m", plane_max_offset_update_m_);
        get_parameter("plane_min_points", plane_min_points_);
        get_parameter("plane_pixel_stride", plane_pixel_stride_);
        load_layout();
        validate_depth_refinement_parameters();

        EeMarker primary_ee_marker;
        get_parameter("ee_marker_id", primary_ee_marker.id);
        get_parameter("ee_marker_frame", primary_ee_marker.frame);
        ee_markers_.push_back(std::move(primary_ee_marker));

        EeMarker secondary_ee_marker;
        get_parameter("secondary_ee_marker_id", secondary_ee_marker.id);
        get_parameter(
            "secondary_ee_marker_frame", secondary_ee_marker.frame
        );
        ee_markers_.push_back(std::move(secondary_ee_marker));

        ema_alpha_ = std::clamp(ema_alpha_, 1e-6, 1.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_pub_ = create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>("panel_pose", 10);
        visual_ee_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "visual_ee_pose", 10
        );

        using namespace std::placeholders;
        set_marker_ids_service_ = create_service<SetMarkerIds>(
            "set_panel_marker_ids",
            std::bind(&PanelTracker::set_marker_ids, this, _1, _2)
        );
        calibration_action_server_ =
            rclcpp_action::create_server<CalibrateAction>(
                this,
                "calibrate_panel_marker_ids",
                std::bind(&PanelTracker::handle_calibration_goal, this, _1, _2),
                std::bind(&PanelTracker::handle_calibration_cancel, this, _1),
                std::bind(&PanelTracker::handle_calibration_accepted, this, _1)
            );
        calibration_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PanelTracker::check_calibration_deadline, this)
        );

        detection_sub_ =
            create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                detection_topic_,
                10,
                std::bind(
                    &PanelTracker::on_detection,
                    this,
                    std::placeholders::_1
                )
            );
        const auto sensor_qos = rclcpp::SensorDataQoS();
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_,
            sensor_qos,
            std::bind(&PanelTracker::on_depth, this, std::placeholders::_1)
        );
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_,
            sensor_qos,
            std::bind(&PanelTracker::on_camera_info, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            get_logger(),
            "PanelTracker tracking '%s' in frame '%s' and publishing '%s'",
            detection_topic_.c_str(),
            tracking_frame_.c_str(),
            board_frame_.c_str()
        );
    }

  private:
    struct EeMarker {
        int id{0};
        std::string frame;
        std::optional<tf2::Transform> cached_marker_to_ee;
    };

    struct MarkerLayout {
        std::string name;
        int id;
        double u;
        double v;
        double yaw_rad;
    };

    struct DepthFrame {
        rclcpp::Time stamp;
        cv::Mat meters;
    };

    struct CameraIntrinsics {
        int width;
        int height;
        double focal_x;
        double focal_y;
        double center_x;
        double center_y;
    };

    struct PlaneFit {
        Eigen::Vector3d normal;
        double offset;
        double rms;
        size_t inlier_count;
    };

    std::string layout_yaml_path_;
    std::string tracking_frame_;
    std::string board_frame_;
    std::string detection_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string ee_frame_;
    std::vector<EeMarker> ee_markers_;
    double ema_alpha_;
    double board_width_{0.0};
    double board_height_{0.0};
    std::vector<MarkerLayout> markers_;
    std::set<int> allowed_marker_ids_;
    int default_calibration_confirmations_{5};
    double default_calibration_timeout_seconds_{10.0};
    double calibration_max_distance_error_{0.05};
    bool depth_refinement_enabled_{true};
    double depth_max_age_s_{0.1};
    double plane_roi_margin_m_{0.02};
    double plane_initial_distance_m_{0.02};
    double plane_min_residual_threshold_m_{0.002};
    double plane_robust_sigma_multiplier_{3.0};
    int plane_iterations_{3};
    double plane_max_normal_update_rad_{0.0872664626};
    double plane_max_offset_update_m_{0.01};
    int plane_min_points_{200};
    int plane_pixel_stride_{2};
    std::optional<DepthFrame> latest_depth_;
    std::optional<CameraIntrinsics> camera_intrinsics_;
    bool filter_initialized_{false};
    tf2::Transform filtered_board_pose_;

    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_ee_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Service<SetMarkerIds>::SharedPtr set_marker_ids_service_;
    rclcpp_action::Server<CalibrateAction>::SharedPtr calibration_action_server_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;
    std::shared_ptr<CalibrationGoalHandle> calibration_goal_;
    bool calibration_goal_reserved_{false};
    std::optional<std::vector<int>> calibration_candidate_;
    uint32_t calibration_confirmations_{0};
    uint32_t required_calibration_confirmations_{0};
    rclcpp::Time calibration_deadline_;

    void load_layout() {
        const YAML::Node config = YAML::LoadFile(layout_yaml_path_);
        board_width_ = config["board_width"].as<double>();
        board_height_ = config["board_height"].as<double>();

        allowed_marker_ids_.clear();
        for (const auto &id : config["allowed_marker_ids"]) {
            allowed_marker_ids_.insert(id.as<int>());
        }
        if (allowed_marker_ids_.empty()) {
            throw std::runtime_error("allowed_marker_ids must not be empty");
        }

        markers_.clear();
        std::set<int> assigned_ids;
        for (const auto &entry : config["markers"]) {
            MarkerLayout marker;
            marker.name = entry.first.as<std::string>();
            marker.id = entry.second["id"].as<int>();
            marker.u = entry.second["u"].as<double>();
            marker.v = entry.second["v"].as<double>();
            marker.yaw_rad = entry.second["yaw_deg"].as<double>(0.0) *
                             M_PI / 180.0;
            if (!allowed_marker_ids_.contains(marker.id)) {
                throw std::runtime_error(
                    "Marker slot '" + marker.name + "' uses disallowed ID " +
                    std::to_string(marker.id)
                );
            }
            if (!assigned_ids.insert(marker.id).second) {
                throw std::runtime_error(
                    "Marker ID " + std::to_string(marker.id) +
                    " is assigned more than once"
                );
            }
            markers_.push_back(std::move(marker));
        }
        if (markers_.size() < 3) {
            throw std::runtime_error(
                "At least three marker slots are required for calibration"
            );
        }
        default_calibration_confirmations_ =
            std::max(1, default_calibration_confirmations_);
        if (default_calibration_timeout_seconds_ <= 0.0) {
            default_calibration_timeout_seconds_ = 10.0;
        }
        if (calibration_max_distance_error_ <= 0.0) {
            calibration_max_distance_error_ = 0.05;
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %zu panel marker slots and %zu allowed IDs from %s",
            markers_.size(),
            allowed_marker_ids_.size(),
            layout_yaml_path_.c_str()
        );
    }

    void validate_depth_refinement_parameters() const {
        if (!std::isfinite(depth_max_age_s_) || depth_max_age_s_ <= 0.0 ||
            !std::isfinite(plane_roi_margin_m_) || plane_roi_margin_m_ < 0.0 ||
            plane_roi_margin_m_ * 2.0 >=
                std::min(board_width_, board_height_) ||
            !std::isfinite(plane_initial_distance_m_) ||
            plane_initial_distance_m_ <= 0.0 ||
            !std::isfinite(plane_min_residual_threshold_m_) ||
            plane_min_residual_threshold_m_ <= 0.0 ||
            !std::isfinite(plane_robust_sigma_multiplier_) ||
            plane_robust_sigma_multiplier_ <= 0.0 ||
            plane_iterations_ <= 0 ||
            !std::isfinite(plane_max_normal_update_rad_) ||
            plane_max_normal_update_rad_ <= 0.0 ||
            !std::isfinite(plane_max_offset_update_m_) ||
            plane_max_offset_update_m_ <= 0.0 ||
            plane_min_points_ < 3 || plane_pixel_stride_ <= 0) {
            throw std::invalid_argument("Invalid panel depth-refinement parameters");
        }
    }

    void on_camera_info(
        const sensor_msgs::msg::CameraInfo::SharedPtr message
    ) {
        if (message->width == 0 || message->height == 0 ||
            message->k[0] <= 0.0 || message->k[4] <= 0.0) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid panel camera calibration");
            return;
        }
        camera_intrinsics_ = CameraIntrinsics{
            static_cast<int>(message->width),
            static_cast<int>(message->height),
            message->k[0],
            message->k[4],
            message->k[2],
            message->k[5],
        };
    }

    void on_depth(const sensor_msgs::msg::Image::SharedPtr message) {
        try {
            const auto image = cv_bridge::toCvShare(message);
            cv::Mat meters;
            if (message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                message->encoding == sensor_msgs::image_encodings::MONO16) {
                image->image.convertTo(meters, CV_32FC1, 0.001);
            } else if (
                message->encoding == sensor_msgs::image_encodings::TYPE_32FC1
            ) {
                meters = image->image.clone();
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Unsupported panel depth encoding '%s'",
                    message->encoding.c_str()
                );
                return;
            }
            latest_depth_ = DepthFrame{
                rclcpp::Time(message->header.stamp), std::move(meters)
            };
        } catch (const std::exception &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Failed to decode panel depth: %s",
                error.what()
            );
        }
    }

    static double median(std::vector<double> values) {
        if (values.empty()) {
            return 0.0;
        }
        const size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        const double upper = values[middle];
        if (values.size() % 2 != 0) {
            return upper;
        }
        std::nth_element(
            values.begin(), values.begin() + middle - 1, values.begin() + middle
        );
        return 0.5 * (values[middle - 1] + upper);
    }

    std::optional<PlaneFit> fit_panel_plane(
        const std::vector<Eigen::Vector3d> &points,
        const Eigen::Vector3d &prior_normal
    ) const {
        std::vector<size_t> inliers(points.size());
        for (size_t index = 0; index < points.size(); ++index) {
            inliers[index] = index;
        }

        PlaneFit fit{};
        for (int iteration = 0; iteration < plane_iterations_; ++iteration) {
            if (inliers.size() < static_cast<size_t>(plane_min_points_)) {
                return std::nullopt;
            }

            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const size_t index : inliers) {
                centroid += points[index];
            }
            centroid /= static_cast<double>(inliers.size());

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const size_t index : inliers) {
                const Eigen::Vector3d centered = points[index] - centroid;
                covariance += centered * centered.transpose();
            }
            covariance /= static_cast<double>(inliers.size());

            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
                covariance
            );
            if (solver.info() != Eigen::Success ||
                solver.eigenvalues()[1] <= 1e-10) {
                return std::nullopt;
            }
            fit.normal = solver.eigenvectors().col(0).normalized();
            if (fit.normal.dot(prior_normal) < 0.0) {
                fit.normal = -fit.normal;
            }
            fit.offset = -fit.normal.dot(centroid);
            fit.inlier_count = inliers.size();

            std::vector<double> absolute_residuals;
            absolute_residuals.reserve(inliers.size());
            double squared_residual = 0.0;
            for (const size_t index : inliers) {
                const double residual =
                    fit.normal.dot(points[index]) + fit.offset;
                absolute_residuals.push_back(std::abs(residual));
                squared_residual += residual * residual;
            }
            fit.rms = std::sqrt(
                squared_residual / static_cast<double>(inliers.size())
            );

            if (iteration + 1 >= plane_iterations_) {
                break;
            }
            const double robust_sigma =
                1.4826 * median(std::move(absolute_residuals));
            const double threshold = std::max(
                plane_min_residual_threshold_m_,
                plane_robust_sigma_multiplier_ * robust_sigma
            );
            std::vector<size_t> trimmed;
            trimmed.reserve(inliers.size());
            for (const size_t index : inliers) {
                if (std::abs(fit.normal.dot(points[index]) + fit.offset) <=
                    threshold) {
                    trimmed.push_back(index);
                }
            }
            inliers = std::move(trimmed);
        }
        return fit;
    }

    std::optional<tf2::Transform> refine_with_depth(
        const tf2::Transform &camera_to_board,
        const rclcpp::Time &detection_stamp
    ) {
        if (!depth_refinement_enabled_ || !latest_depth_ ||
            !camera_intrinsics_) {
            return std::nullopt;
        }
        const double depth_age =
            std::abs((detection_stamp - latest_depth_->stamp).seconds());
        if (depth_age > depth_max_age_s_) {
            return std::nullopt;
        }

        const cv::Mat &depth = latest_depth_->meters;
        const CameraIntrinsics &intrinsics = *camera_intrinsics_;
        if (depth.cols != intrinsics.width || depth.rows != intrinsics.height) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Depth dimensions %dx%d do not match camera info %dx%d",
                depth.cols,
                depth.rows,
                intrinsics.width,
                intrinsics.height
            );
            return std::nullopt;
        }

        const tf2::Transform board_to_camera = camera_to_board.inverse();
        std::vector<Eigen::Vector3d> candidates;
        candidates.reserve(
            static_cast<size_t>(depth.rows * depth.cols) /
            static_cast<size_t>(plane_pixel_stride_ * plane_pixel_stride_)
        );
        const double minimum_x = -0.5 * board_width_ + plane_roi_margin_m_;
        const double maximum_x = 0.5 * board_width_ - plane_roi_margin_m_;
        const double minimum_y = -0.5 * board_height_ + plane_roi_margin_m_;
        const double maximum_y = 0.5 * board_height_ - plane_roi_margin_m_;
        for (int row = 0; row < depth.rows; row += plane_pixel_stride_) {
            const float *depth_values = depth.ptr<float>(row);
            for (int column = 0; column < depth.cols;
                 column += plane_pixel_stride_) {
                const float range = depth_values[column];
                if (!std::isfinite(range) || range <= 0.0F) {
                    continue;
                }
                const tf2::Vector3 camera_point(
                    (static_cast<double>(column) - intrinsics.center_x) /
                        intrinsics.focal_x * range,
                    (static_cast<double>(row) - intrinsics.center_y) /
                        intrinsics.focal_y * range,
                    range
                );
                const tf2::Vector3 board_point =
                    board_to_camera * camera_point;
                if (board_point.x() < minimum_x ||
                    board_point.x() > maximum_x ||
                    board_point.y() < minimum_y ||
                    board_point.y() > maximum_y ||
                    std::abs(board_point.z()) > plane_initial_distance_m_) {
                    continue;
                }
                candidates.emplace_back(
                    camera_point.x(), camera_point.y(), camera_point.z()
                );
            }
        }
        if (candidates.size() < static_cast<size_t>(plane_min_points_)) {
            return std::nullopt;
        }

        const tf2::Vector3 prior_normal_tf =
            camera_to_board.getBasis().getColumn(2).normalized();
        const Eigen::Vector3d prior_normal(
            prior_normal_tf.x(), prior_normal_tf.y(), prior_normal_tf.z()
        );
        const auto plane = fit_panel_plane(candidates, prior_normal);
        if (!plane) {
            return std::nullopt;
        }

        const double normal_angle = std::acos(std::clamp(
            prior_normal.dot(plane->normal), -1.0, 1.0
        ));
        const tf2::Vector3 prior_origin_tf = camera_to_board.getOrigin();
        const Eigen::Vector3d prior_origin(
            prior_origin_tf.x(), prior_origin_tf.y(), prior_origin_tf.z()
        );
        const double signed_offset =
            plane->normal.dot(prior_origin) + plane->offset;
        if (normal_angle > plane_max_normal_update_rad_ ||
            std::abs(signed_offset) > plane_max_offset_update_m_) {
            return std::nullopt;
        }

        tf2::Transform refined = camera_to_board;
        tf2::Quaternion normal_correction = tf2::Quaternion::getIdentity();
        const tf2::Vector3 fitted_normal(
            plane->normal.x(), plane->normal.y(), plane->normal.z()
        );
        tf2::Vector3 rotation_axis = prior_normal_tf.cross(fitted_normal);
        if (rotation_axis.length2() > 1e-12) {
            rotation_axis.normalize();
            normal_correction.setRotation(rotation_axis, normal_angle);
        }
        tf2::Quaternion refined_rotation =
            normal_correction * camera_to_board.getRotation();
        refined_rotation.normalize();
        refined.setRotation(refined_rotation);
        const Eigen::Vector3d refined_origin =
            prior_origin - signed_offset * plane->normal;
        refined.setOrigin(tf2::Vector3(
            refined_origin.x(), refined_origin.y(), refined_origin.z()
        ));

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Depth plane refined panel from %zu/%zu points: rms=%.4f m, "
            "normal_update=%.2f deg, offset=%.4f m",
            plane->inlier_count,
            candidates.size(),
            plane->rms,
            normal_angle * 180.0 / M_PI,
            signed_offset
        );
        return refined;
    }

    bool apply_marker_ids(
        const std::vector<std::string> &names,
        const std::vector<int32_t> &ids,
        std::string &error
    ) {
        if (names.size() != markers_.size() || ids.size() != markers_.size()) {
            error = "Assignment must contain every configured marker slot";
            return false;
        }

        std::map<std::string, int> assignment;
        std::set<int> unique_ids;
        for (size_t index = 0; index < names.size(); ++index) {
            if (!assignment.emplace(names[index], ids[index]).second) {
                error = "Marker slot '" + names[index] + "' occurs more than once";
                return false;
            }
            if (!allowed_marker_ids_.contains(ids[index])) {
                error = "Marker ID " + std::to_string(ids[index]) +
                        " is not allowed on the panel";
                return false;
            }
            if (!unique_ids.insert(ids[index]).second) {
                error = "Marker ID " + std::to_string(ids[index]) +
                        " occurs more than once";
                return false;
            }
        }

        for (const auto &marker : markers_) {
            if (!assignment.contains(marker.name)) {
                error = "Unknown or missing marker slot '" + marker.name + "'";
                return false;
            }
        }
        for (const auto &[name, id] : assignment) {
            const auto known_slot = std::find_if(
                markers_.begin(),
                markers_.end(),
                [&name](const MarkerLayout &marker) { return marker.name == name; }
            );
            if (known_slot == markers_.end()) {
                error = "Unknown marker slot '" + name + "'";
                return false;
            }
            known_slot->id = id;
        }

        filter_initialized_ = false;
        RCLCPP_INFO(get_logger(), "Updated panel marker ID assignment");
        return true;
    }

    void set_marker_ids(
        const std::shared_ptr<SetMarkerIds::Request> request,
        std::shared_ptr<SetMarkerIds::Response> response
    ) {
        if (calibration_goal_ || calibration_goal_reserved_) {
            response->result = false;
            response->message = "Cannot set marker IDs while calibration is active";
            return;
        }
        response->result = apply_marker_ids(
            request->marker_names, request->marker_ids, response->message
        );
        if (response->result) {
            response->message = "Panel marker IDs updated";
        }
    }

    rclcpp_action::GoalResponse handle_calibration_goal(
        const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const CalibrateAction::Goal>
    ) {
        if (calibration_goal_ || calibration_goal_reserved_) {
            RCLCPP_WARN(get_logger(), "Rejecting marker calibration: already active");
            return rclcpp_action::GoalResponse::REJECT;
        }
        calibration_goal_reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_calibration_cancel(
        const std::shared_ptr<CalibrationGoalHandle>
    ) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_calibration_accepted(
        const std::shared_ptr<CalibrationGoalHandle> goal_handle
    ) {
        calibration_goal_ = goal_handle;
        calibration_goal_reserved_ = false;
        calibration_candidate_.reset();
        calibration_confirmations_ = 0;
        const auto &goal = *goal_handle->get_goal();
        required_calibration_confirmations_ = goal.required_confirmations > 0
            ? goal.required_confirmations
            : static_cast<uint32_t>(default_calibration_confirmations_);
        const double timeout = goal.timeout_seconds > 0.0
            ? goal.timeout_seconds
            : default_calibration_timeout_seconds_;
        calibration_deadline_ =
            now() + rclcpp::Duration::from_seconds(timeout);
        RCLCPP_INFO(
            get_logger(),
            "Started panel marker calibration; requiring %u confirmations",
            required_calibration_confirmations_
        );
    }

    void fill_calibration_result(CalibrateAction::Result &result) const {
        for (const auto &marker : markers_) {
            result.marker_names.push_back(marker.name);
            result.marker_ids.push_back(marker.id);
        }
    }

    void check_calibration_deadline() {
        if (!calibration_goal_) {
            return;
        }
        if (calibration_goal_->is_canceling()) {
            auto result = std::make_shared<CalibrateAction::Result>();
            result->result = false;
            result->message = "Panel marker calibration canceled";
            fill_calibration_result(*result);
            calibration_goal_->canceled(result);
            calibration_goal_.reset();
            calibration_goal_reserved_ = false;
            calibration_candidate_.reset();
            return;
        }
        if (now() >= calibration_deadline_) {
            auto result = std::make_shared<CalibrateAction::Result>();
            result->result = false;
            result->message = "Panel marker calibration timed out";
            fill_calibration_result(*result);
            calibration_goal_->abort(result);
            calibration_goal_.reset();
            calibration_goal_reserved_ = false;
            calibration_candidate_.reset();
        }
    }

    std::optional<std::vector<int>> infer_marker_ids(
        const aruco_opencv_msgs::msg::ArucoDetection &detection
    ) const {
        using MarkerPose = aruco_opencv_msgs::msg::MarkerPose;
        std::vector<const MarkerPose *> observed;
        std::set<int> observed_ids;
        for (const auto &marker : detection.markers) {
            if (allowed_marker_ids_.contains(marker.marker_id) &&
                observed_ids.insert(marker.marker_id).second) {
                observed.push_back(&marker);
            }
        }
        if (observed.size() < markers_.size()) {
            return std::nullopt;
        }

        auto observed_distance = [](const MarkerPose &left, const MarkerPose &right) {
            const double dx = left.pose.position.x - right.pose.position.x;
            const double dy = left.pose.position.y - right.pose.position.y;
            const double dz = left.pose.position.z - right.pose.position.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        };
        auto expected_distance = [this](size_t left, size_t right) {
            const double du = markers_[left].u - markers_[right].u;
            const double dv = markers_[left].v - markers_[right].v;
            return std::sqrt(du * du + dv * dv);
        };

        double best_error = std::numeric_limits<double>::infinity();
        std::vector<int> best_assignment;
        std::vector<size_t> assignment(markers_.size());
        std::vector<bool> used(observed.size(), false);
        std::function<void(size_t)> search = [&](size_t slot_index) {
            if (slot_index == markers_.size()) {
                double squared_error = 0.0;
                size_t pair_count = 0;
                for (size_t left = 0; left < markers_.size(); ++left) {
                    for (size_t right = left + 1; right < markers_.size(); ++right) {
                        const double error =
                            observed_distance(
                                *observed[assignment[left]],
                                *observed[assignment[right]]
                            ) - expected_distance(left, right);
                        squared_error += error * error;
                        ++pair_count;
                    }
                }
                const double rms_error =
                    std::sqrt(squared_error / static_cast<double>(pair_count));
                if (rms_error < best_error) {
                    best_error = rms_error;
                    best_assignment.clear();
                    for (const size_t observed_index : assignment) {
                        best_assignment.push_back(
                            observed[observed_index]->marker_id
                        );
                    }
                }
                return;
            }

            for (size_t observed_index = 0;
                 observed_index < observed.size();
                 ++observed_index) {
                if (used[observed_index]) {
                    continue;
                }
                used[observed_index] = true;
                assignment[slot_index] = observed_index;
                search(slot_index + 1);
                used[observed_index] = false;
            }
        };
        search(0);

        if (best_error > calibration_max_distance_error_) {
            return std::nullopt;
        }
        return best_assignment;
    }

    void process_calibration(
        const aruco_opencv_msgs::msg::ArucoDetection &detection
    ) {
        if (!calibration_goal_ || !calibration_goal_->is_active()) {
            return;
        }
        const auto inferred = infer_marker_ids(detection);
        if (!inferred) {
            auto feedback = std::make_shared<CalibrateAction::Feedback>();
            feedback->confirmations = 0;
            feedback->progress = "Waiting for a complete geometry match";
            calibration_goal_->publish_feedback(feedback);
            calibration_candidate_.reset();
            calibration_confirmations_ = 0;
            return;
        }

        if (calibration_candidate_ && *calibration_candidate_ == *inferred) {
            ++calibration_confirmations_;
        } else {
            calibration_candidate_ = inferred;
            calibration_confirmations_ = 1;
        }

        auto feedback = std::make_shared<CalibrateAction::Feedback>();
        feedback->confirmations = calibration_confirmations_;
        feedback->progress =
            "Confirmed assignment in " +
            std::to_string(calibration_confirmations_) + "/" +
            std::to_string(required_calibration_confirmations_) + " frames";
        calibration_goal_->publish_feedback(feedback);

        if (calibration_confirmations_ < required_calibration_confirmations_) {
            return;
        }

        std::vector<std::string> names;
        std::vector<int32_t> ids;
        for (size_t index = 0; index < markers_.size(); ++index) {
            names.push_back(markers_[index].name);
            ids.push_back(inferred->at(index));
        }
        std::string error;
        auto result = std::make_shared<CalibrateAction::Result>();
        result->result = apply_marker_ids(names, ids, error);
        result->message = result->result
            ? "Panel marker IDs calibrated"
            : error;
        fill_calibration_result(*result);
        if (result->result) {
            calibration_goal_->succeed(result);
        } else {
            calibration_goal_->abort(result);
        }
        calibration_goal_.reset();
        calibration_goal_reserved_ = false;
        calibration_candidate_.reset();
    }

    std::optional<tf2::Transform> board_to_marker_from_layout(int marker_id) const {
        for (const auto &marker : markers_) {
            if (marker.id != marker_id) {
                continue;
            }

            tf2::Transform transform;
            transform.setIdentity();
            transform.setOrigin(
                tf2::Vector3(
                    marker.u - board_width_ * 0.5,
                    -(marker.v - board_height_ * 0.5),
                    0.0
                )
            );
            tf2::Quaternion rotation;
            rotation.setRPY(0.0, 0.0, marker.yaw_rad);
            transform.setRotation(rotation);
            return transform;
        }

        return std::nullopt;
    }

    tf2::Transform average_transforms(
        const std::vector<tf2::Transform> &transforms
    ) const {
        tf2::Vector3 mean_translation(0.0, 0.0, 0.0);
        tf2::Quaternion mean_rotation(
            transforms.front().getRotation().x(),
            transforms.front().getRotation().y(),
            transforms.front().getRotation().z(),
            transforms.front().getRotation().w()
        );

        for (const auto &transform : transforms) {
            mean_translation += transform.getOrigin();
        }
        mean_translation /= static_cast<double>(transforms.size());

        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 0.0;
        const tf2::Quaternion reference = transforms.front().getRotation();
        for (const auto &transform : transforms) {
            tf2::Quaternion q = transform.getRotation();
            if (reference.dot(q) < 0.0) {
                q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
            }
            qx += q.x();
            qy += q.y();
            qz += q.z();
            qw += q.w();
        }

        mean_rotation = tf2::Quaternion(qx, qy, qz, qw);
        mean_rotation.normalize();

        tf2::Transform averaged;
        averaged.setOrigin(mean_translation);
        averaged.setRotation(mean_rotation);
        return averaged;
    }

    tf2::Transform apply_ema(const tf2::Transform &measurement) {
        if (!filter_initialized_) {
            filtered_board_pose_ = measurement;
            filter_initialized_ = true;
            return filtered_board_pose_;
        }

        const tf2::Vector3 translation =
            (1.0 - ema_alpha_) * filtered_board_pose_.getOrigin() +
            ema_alpha_ * measurement.getOrigin();

        tf2::Quaternion current = filtered_board_pose_.getRotation();
        tf2::Quaternion incoming = measurement.getRotation();
        if (current.dot(incoming) < 0.0) {
            incoming = tf2::Quaternion(
                -incoming.x(),
                -incoming.y(),
                -incoming.z(),
                -incoming.w()
            );
        }

        tf2::Quaternion rotation = current.slerp(incoming, ema_alpha_);
        rotation.normalize();

        filtered_board_pose_.setOrigin(translation);
        filtered_board_pose_.setRotation(rotation);
        return filtered_board_pose_;
    }

    std::array<double, 36> estimate_covariance(
        const std::vector<tf2::Transform> &board_estimates,
        const tf2::Transform &mean_board_pose,
        const std::vector<aruco_opencv_msgs::msg::MarkerPose> &used_markers
    ) const {
        std::array<double, 36> covariance{};
        covariance.fill(0.0);

        const size_t count = board_estimates.size();
        const double marker_count_scale =
            1.0 / std::sqrt(static_cast<double>(std::max<size_t>(1, count)));

        double mean_distance = 0.0;
        for (const auto &marker : used_markers) {
            mean_distance += std::sqrt(
                marker.pose.position.x * marker.pose.position.x +
                marker.pose.position.y * marker.pose.position.y +
                marker.pose.position.z * marker.pose.position.z
            );
        }
        mean_distance /=
            static_cast<double>(std::max<size_t>(1, used_markers.size()));
        const double distance_scale = std::max(0.2, mean_distance);

        double tx_var = 0.0;
        double ty_var = 0.0;
        double tz_var = 0.0;
        double rx_var = 0.0;
        double ry_var = 0.0;
        double rz_var = 0.0;

        for (const auto &estimate : board_estimates) {
            const tf2::Vector3 dt =
                estimate.getOrigin() - mean_board_pose.getOrigin();
            tx_var += dt.x() * dt.x();
            ty_var += dt.y() * dt.y();
            tz_var += dt.z() * dt.z();

            tf2::Quaternion q_err =
                mean_board_pose.getRotation().inverse() * estimate.getRotation();
            q_err.normalize();
            if (q_err.getW() < 0.0) {
                q_err = tf2::Quaternion(
                    -q_err.x(), -q_err.y(), -q_err.z(), -q_err.w()
                );
            }

            const double w =
                std::clamp(static_cast<double>(q_err.getW()), -1.0, 1.0);
            const double angle = 2.0 * std::acos(w);
            const double s = std::sqrt(std::max(1e-16, 1.0 - w * w));
            tf2::Vector3 axis(1.0, 0.0, 0.0);
            if (s > 1e-8) {
                axis = tf2::Vector3(
                    q_err.getX() / s, q_err.getY() / s, q_err.getZ() / s
                );
            }
            const tf2::Vector3 rotvec = axis * angle;
            rx_var += rotvec.x() * rotvec.x();
            ry_var += rotvec.y() * rotvec.y();
            rz_var += rotvec.z() * rotvec.z();
        }

        const double denom = static_cast<double>(std::max<size_t>(1, count - 1));
        tx_var /= denom;
        ty_var /= denom;
        tz_var /= denom;
        rx_var /= denom;
        ry_var /= denom;
        rz_var /= denom;

        const double pos_floor =
            std::pow(0.005 * distance_scale * marker_count_scale, 2);
        const double rot_floor =
            std::pow(0.03 * distance_scale * marker_count_scale, 2);

        covariance[0] = tx_var + pos_floor;
        covariance[7] = ty_var + pos_floor;
        covariance[14] = tz_var + pos_floor * 2.0;
        covariance[21] = rx_var + rot_floor;
        covariance[28] = ry_var + rot_floor;
        covariance[35] = rz_var + rot_floor;
        return covariance;
    }

    std::optional<tf2::Transform> marker_to_ee(EeMarker &marker) {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                marker.frame,
                ee_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1)
            );
            tf2::Transform current;
            tf2::fromMsg(transform.transform, current);

            if (marker.cached_marker_to_ee) {
                const tf2::Transform delta =
                    marker.cached_marker_to_ee->inverse() * current;
                const double translation_change = delta.getOrigin().length();
                tf2::Quaternion rotation_change = delta.getRotation();
                rotation_change.normalize();
                const double rotation_angle = 2.0 * std::acos(std::clamp(
                    std::abs(static_cast<double>(rotation_change.w())), 0.0, 1.0
                ));
                if (translation_change > 1e-5 || rotation_angle > 1e-4) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "Transform %s -> %s is not static (delta %.6f m, %.6f rad)",
                        marker.frame.c_str(),
                        ee_frame_.c_str(),
                        translation_change,
                        rotation_angle
                    );
                }
            } else {
                marker.cached_marker_to_ee = current;
            }
            return current;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Could not resolve %s -> %s: %s",
                marker.frame.c_str(),
                ee_frame_.c_str(),
                ex.what()
            );
            return std::nullopt;
        }
    }

    void on_detection(
        const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg
    ) {
        if (msg->markers.empty()) {
            return;
        }
        process_calibration(*msg);

        const rclcpp::Time stamp = msg->header.stamp;
        const std::string camera_frame = msg->header.frame_id;

        geometry_msgs::msg::TransformStamped tracking_to_camera;
        try {
            tracking_to_camera = tf_buffer_->lookupTransform(
                tracking_frame_,
                camera_frame,
                stamp,
                rclcpp::Duration::from_seconds(0.1)
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Could not transform '%s' to '%s': %s",
                camera_frame.c_str(),
                tracking_frame_.c_str(),
                ex.what()
            );
            return;
        }

        tf2::Transform t_tracking_camera;
        tf2::fromMsg(tracking_to_camera.transform, t_tracking_camera);

        std::vector<tf2::Transform> board_estimates;
        std::vector<int> marker_ids;
        std::vector<aruco_opencv_msgs::msg::MarkerPose> used_markers;
        std::vector<tf2::Transform> camera_to_ee_estimates;
        for (const auto &marker : msg->markers) {
            geometry_msgs::msg::Transform marker_transform_msg;
            marker_transform_msg.translation.x = marker.pose.position.x;
            marker_transform_msg.translation.y = marker.pose.position.y;
            marker_transform_msg.translation.z = marker.pose.position.z;
            marker_transform_msg.rotation = marker.pose.orientation;

            tf2::Transform t_camera_marker;
            tf2::fromMsg(marker_transform_msg, t_camera_marker);

            const auto ee_marker = std::find_if(
                ee_markers_.begin(),
                ee_markers_.end(),
                [&marker](const EeMarker &candidate) {
                    return candidate.id == marker.marker_id;
                }
            );
            if (ee_marker != ee_markers_.end()) {
                const auto marker_to_end_effector = marker_to_ee(*ee_marker);
                if (marker_to_end_effector) {
                    camera_to_ee_estimates.push_back(
                        t_camera_marker * *marker_to_end_effector
                    );
                }
            }

            const auto board_to_marker =
                board_to_marker_from_layout(marker.marker_id);
            if (!board_to_marker.has_value()) {
                continue;
            }

            board_estimates.push_back(
                t_camera_marker * board_to_marker->inverse()
            );
            marker_ids.push_back(marker.marker_id);
            used_markers.push_back(marker);
        }

        if (board_estimates.empty()) {
            return;
        }

        const tf2::Transform averaged_camera_to_board =
            average_transforms(board_estimates);
        const tf2::Transform camera_to_board =
            refine_with_depth(averaged_camera_to_board, stamp)
                .value_or(averaged_camera_to_board);
        const tf2::Transform tracking_to_board_measurement =
            t_tracking_camera * camera_to_board;
        const tf2::Transform filtered_tracking_board =
            apply_ema(tracking_to_board_measurement);
        const tf2::Transform filtered_camera_to_board =
            t_tracking_camera.inverse() * filtered_tracking_board;
        const auto covariance = estimate_covariance(
            board_estimates, averaged_camera_to_board, used_markers
        );

        geometry_msgs::msg::TransformStamped board_tf;
        board_tf.header.stamp = stamp;
        board_tf.header.frame_id = tracking_frame_;
        board_tf.child_frame_id = board_frame_;
        board_tf.transform = tf2::toMsg(filtered_tracking_board);
        tf_broadcaster_->sendTransform(board_tf);

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = board_tf.header;
        pose_msg.pose.pose.position.x = board_tf.transform.translation.x;
        pose_msg.pose.pose.position.y = board_tf.transform.translation.y;
        pose_msg.pose.pose.position.z = board_tf.transform.translation.z;
        pose_msg.pose.pose.orientation = board_tf.transform.rotation;
        pose_msg.pose.covariance = covariance;
        pose_pub_->publish(pose_msg);

        if (!camera_to_ee_estimates.empty()) {
            const tf2::Transform camera_to_ee =
                average_transforms(camera_to_ee_estimates);
            const tf2::Transform panel_to_ee =
                filtered_camera_to_board.inverse() * camera_to_ee;
            geometry_msgs::msg::PoseStamped visual_ee;
            visual_ee.header.stamp = stamp;
            visual_ee.header.frame_id = board_frame_;
            visual_ee.pose.position.x = panel_to_ee.getOrigin().x();
            visual_ee.pose.position.y = panel_to_ee.getOrigin().y();
            visual_ee.pose.position.z = panel_to_ee.getOrigin().z();
            visual_ee.pose.orientation = tf2::toMsg(
                panel_to_ee.getRotation()
            );
            visual_ee_pub_->publish(visual_ee);
        }

        RCLCPP_DEBUG(
            get_logger(),
            "Updated board pose from %zu markers in '%s'; visual EE from %zu "
            "marker(s)",
            marker_ids.size(),
            tracking_frame_.c_str(),
            camera_to_ee_estimates.size()
        );
    }
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelTracker)
