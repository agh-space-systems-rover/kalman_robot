#include "actions/find_panel.hpp"

#include "tf_util.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr double kDegToRad = M_PI / 180.0;

geometry_msgs::msg::Pose transform_pose(
    const geometry_msgs::msg::Pose &pose, const tf2::Transform &transform
) {
    geometry_msgs::msg::TransformStamped stamped;
    stamped.transform = tf2::toMsg(transform);
    geometry_msgs::msg::Pose out;
    tf2::doTransform(pose, out, stamped);
    return out;
}
} // namespace

FindPanel::FindPanel(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SyncActionNode(name, config), parent_(parent) {
    panel_sub_ =
        parent_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "panel_pose",
            10,
            std::bind(&FindPanel::panel_pose_callback, this, std::placeholders::_1)
        );
    const auto camera_info_topic =
        getInput<std::string>("camera_info_topic").value_or(
            "/d455_arm/color/camera_info"
        );
    camera_info_sub_ =
        parent_->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic,
            10,
            std::bind(&FindPanel::camera_info_callback, this, std::placeholders::_1)
        );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList FindPanel::providedPorts() {
    return {
        BT::InputPort<std::string>("base_frame", "base_link", "Base frame"),
        BT::InputPort<std::string>(
            "camera_frame", "d455_arm_color_optical_frame", "Optical camera frame"
        ),
        BT::InputPort<std::string>(
            "camera_info_topic",
            "/d455_arm/color/camera_info",
            "Camera info topic used for intrinsics"
        ),
        BT::InputPort<std::string>(
            "end_effector_frame", "arm_link_gripper", "Arm end effector frame"
        ),
        BT::InputPort<double>(
            "view_margin", 1.15, "Safety margin applied to board size"
        ),
        BT::InputPort<double>(
            "min_standoff", 0.35, "Minimum camera standoff from panel"
        ),
        BT::InputPort<double>(
            "max_pose_age_ms", 60000.0, "Maximum age of the last known panel pose"
        ),
        BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
    };
}

void FindPanel::panel_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
) {
    last_panel_pose_ = *msg;
}

void FindPanel::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg
) {
    last_camera_info_ = *msg;
}

std::shared_ptr<MissionHelper> FindPanel::mission_helper() const {
    if (!config().blackboard) {
        return nullptr;
    }
    try {
        return config().blackboard->get<std::shared_ptr<MissionHelper>>("state");
    } catch (...) {
        return nullptr;
    }
}

geometry_msgs::msg::PoseWithCovarianceStamped FindPanel::latest_panel_pose_in_base(
    const std::string &base_frame
) const {
    if (!last_panel_pose_) {
        throw std::runtime_error("no panel pose received yet");
    }

    auto pose = *last_panel_pose_;
    if (pose.header.frame_id.empty() || pose.header.frame_id == base_frame) {
        pose.header.frame_id = base_frame;
        return pose;
    }

    const auto transform = tf_buffer_->lookupTransform(
        base_frame,
        pose.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(0.1)
    );

    geometry_msgs::msg::PoseWithCovarianceStamped transformed;
    tf2::doTransform(pose, transformed, transform);
    return transformed;
}

geometry_msgs::msg::Pose FindPanel::compute_target_pose(
    const geometry_msgs::msg::Pose &base_panel_pose
) const {
    const auto camera_frame =
        getInput<std::string>("camera_frame").value_or("d455_arm_color_optical_frame");
    const auto end_effector_frame =
        getInput<std::string>("end_effector_frame").value_or("arm_link_gripper");
    const double view_margin = getInput<double>("view_margin").value_or(1.15);
    const double min_standoff = getInput<double>("min_standoff").value_or(0.35);

    const auto helper = mission_helper();
    if (!helper) {
        throw std::runtime_error("missing mission state on blackboard");
    }
    if (!last_camera_info_) {
        throw std::runtime_error("no camera_info received yet");
    }
    if (last_camera_info_->width == 0 || last_camera_info_->height == 0) {
        throw std::runtime_error("camera_info has invalid image dimensions");
    }

    const double fx = last_camera_info_->k[0];
    const double fy = last_camera_info_->k[4];
    if (fx <= 1e-6 || fy <= 1e-6) {
        throw std::runtime_error("camera_info has invalid focal lengths");
    }

    const double half_width =
        0.5 * helper->state.layout_.board_width * std::max(1.0, view_margin);
    const double half_height =
        0.5 * helper->state.layout_.board_height * std::max(1.0, view_margin);

    const double required_distance_x =
        half_width * (0.5 * static_cast<double>(last_camera_info_->width)) / fx;
    const double required_distance_y =
        half_height * (0.5 * static_cast<double>(last_camera_info_->height)) / fy;
    const double standoff =
        std::max(min_standoff, std::max(required_distance_x, required_distance_y));

    geometry_msgs::msg::Pose desired_camera_pose_panel;
    desired_camera_pose_panel.position.z = standoff;
    desired_camera_pose_panel.orientation.x = 1.0;
    desired_camera_pose_panel.orientation.w = 0.0;

    const tf2::Transform base_to_panel = tf_util::toTf(base_panel_pose);
    const geometry_msgs::msg::Pose desired_camera_pose_base =
        transform_pose(desired_camera_pose_panel, base_to_panel);

    const auto ee_to_camera_msg = tf_buffer_->lookupTransform(
        end_effector_frame,
        camera_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.1)
    );
    const tf2::Transform ee_to_camera = tf_util::toTf(ee_to_camera_msg.transform);
    const tf2::Transform base_to_camera = tf_util::toTf(desired_camera_pose_base);
    const tf2::Transform base_to_ee = base_to_camera * ee_to_camera.inverse();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = base_to_ee.getOrigin().x();
    target_pose.position.y = base_to_ee.getOrigin().y();
    target_pose.position.z = base_to_ee.getOrigin().z();
    target_pose.orientation = tf2::toMsg(base_to_ee.getRotation());
    return target_pose;
}

BT::NodeStatus FindPanel::tick() {
    const auto base_frame =
        getInput<std::string>("base_frame").value_or("base_link");
    const double max_pose_age_ms = getInput<double>("max_pose_age_ms").value_or(60000.0);

    try {
        const auto panel_pose = latest_panel_pose_in_base(base_frame);
        const double age_ms =
            (parent_->now() - rclcpp::Time(panel_pose.header.stamp)).seconds() * 1000.0;
        if (age_ms > max_pose_age_ms) {
            RCLCPP_WARN_STREAM(
                parent_->get_logger(),
                name() << " failed: last panel pose is too old (" << age_ms
                       << " ms > " << max_pose_age_ms << " ms)"
            );
            return BT::NodeStatus::FAILURE;
        }

        const auto target_pose = compute_target_pose(panel_pose.pose.pose);
        setOutput("target_pose", target_pose);
        RCLCPP_INFO_STREAM(
            parent_->get_logger(),
            name() << " computed panel search pose from last known panel pose, age_ms="
                   << age_ms
        );
        return BT::NodeStatus::SUCCESS;
    } catch (const std::exception &e) {
        RCLCPP_WARN_STREAM(parent_->get_logger(), name() << " failed: " << e.what());
        return BT::NodeStatus::FAILURE;
    }
}
