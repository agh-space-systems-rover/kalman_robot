#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace kalman_arm2 {

class PanelTracker : public rclcpp::Node {
  public:
    PanelTracker(const rclcpp::NodeOptions &options)
        : Node("panel_tracker", options) {
        declare_parameter<std::string>("tracking_frame", "odom");
        declare_parameter<std::string>("board_frame", "aruco_board");
        declare_parameter<std::string>(
            "detection_topic", "/d455_arm/aruco_detections"
        );
        declare_parameter<double>("ema_alpha", 0.2);

        get_parameter("tracking_frame", tracking_frame_);
        get_parameter("board_frame", board_frame_);
        get_parameter("detection_topic", detection_topic_);
        get_parameter("ema_alpha", ema_alpha_);

        ema_alpha_ = std::clamp(ema_alpha_, 1e-6, 1.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>("panel_pose", 10);

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

        RCLCPP_INFO(
            get_logger(),
            "PanelTracker tracking '%s' in frame '%s' and publishing '%s'",
            detection_topic_.c_str(),
            tracking_frame_.c_str(),
            board_frame_.c_str()
        );
    }

  private:
    struct MarkerLayout {
        int id;
        double u;
        double v;
    };

    static constexpr std::array<MarkerLayout, 3> kMarkers{{
        {0, 0.0, 0.0},
        {1, 0.26, 0.0},
        {2, 0.0, 0.383},
    }};
    static constexpr double kBoardWidth = 0.4;
    static constexpr double kBoardHeight = 0.6;

    std::string tracking_frame_;
    std::string board_frame_;
    std::string detection_topic_;
    double ema_alpha_;
    bool filter_initialized_{false};
    tf2::Transform filtered_board_pose_;

    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr
        detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::optional<tf2::Transform> board_to_marker_from_layout(int marker_id) const {
        for (const auto &marker : kMarkers) {
            if (marker.id != marker_id) {
                continue;
            }

            tf2::Transform transform;
            transform.setIdentity();
            transform.setOrigin(
                tf2::Vector3(
                    marker.u - kBoardWidth * 0.5,
                    -(marker.v - kBoardHeight * 0.5),
                    0.0
                )
            );
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

    void on_detection(
        const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg
    ) {
        if (msg->markers.empty()) {
            return;
        }

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
        for (const auto &marker : msg->markers) {
            const auto board_to_marker =
                board_to_marker_from_layout(marker.marker_id);
            if (!board_to_marker.has_value()) {
                continue;
            }

            geometry_msgs::msg::Transform marker_transform_msg;
            marker_transform_msg.translation.x = marker.pose.position.x;
            marker_transform_msg.translation.y = marker.pose.position.y;
            marker_transform_msg.translation.z = marker.pose.position.z;
            marker_transform_msg.rotation = marker.pose.orientation;

            tf2::Transform t_camera_marker;
            tf2::fromMsg(marker_transform_msg, t_camera_marker);

            board_estimates.push_back(
                t_tracking_camera * t_camera_marker * board_to_marker->inverse()
            );
            marker_ids.push_back(marker.marker_id);
        }

        if (board_estimates.empty()) {
            return;
        }

        const tf2::Transform averaged_measurement =
            average_transforms(board_estimates);
        const tf2::Transform filtered_measurement =
            apply_ema(averaged_measurement);

        geometry_msgs::msg::TransformStamped board_tf;
        board_tf.header.stamp = stamp;
        board_tf.header.frame_id = tracking_frame_;
        board_tf.child_frame_id = board_frame_;
        board_tf.transform = tf2::toMsg(filtered_measurement);
        tf_broadcaster_->sendTransform(board_tf);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = board_tf.header;
        pose_msg.pose.position.x = board_tf.transform.translation.x;
        pose_msg.pose.position.y = board_tf.transform.translation.y;
        pose_msg.pose.position.z = board_tf.transform.translation.z;
        pose_msg.pose.orientation = board_tf.transform.rotation;
        pose_pub_->publish(pose_msg);

        RCLCPP_DEBUG(
            get_logger(),
            "Updated board pose from %zu markers in '%s'",
            marker_ids.size(),
            tracking_frame_.c_str()
        );
    }
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelTracker)
