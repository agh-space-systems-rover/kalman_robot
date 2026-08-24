#include "actions/pose_ik_navigate_to_pose.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {
tf2::Transform toTf(const geometry_msgs::msg::Transform &tmsg) {
	tf2::Transform t;
	tf2::fromMsg(tmsg, t);
	return t;
}

geometry_msgs::msg::Pose transformPoseWithTf(
    const geometry_msgs::msg::Pose &in, const tf2::Transform &transform
) {
	geometry_msgs::msg::TransformStamped ts;
	ts.transform = tf2::toMsg(transform);

	geometry_msgs::msg::Pose out;
	tf2::doTransform(in, out, ts);
	return out;
}

double quaternionAngleToTarget(
    const geometry_msgs::msg::Quaternion &q_current_msg,
    const geometry_msgs::msg::Quaternion &q_target_msg
) {
	tf2::Quaternion q_c, q_t;
	tf2::fromMsg(q_current_msg, q_c);
	tf2::fromMsg(q_target_msg, q_t);
	q_c.normalize();
	q_t.normalize();

	tf2::Quaternion q_err = q_c.inverse() * q_t;
	q_err.normalize();
	if (q_err.getW() < 0.0) {
		q_err = tf2::Quaternion(
		    -q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW()
		);
	}

	const double w = std::clamp(static_cast<double>(q_err.getW()), -1.0, 1.0);
	return 2.0 * std::acos(w);
}
} // namespace

PoseIKNavigateToPose::PoseIKNavigateToPose(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {
	pose_pub_ = parent_->create_publisher<geometry_msgs::msg::PoseStamped>(
	    "target_pose", 10
	);
	marker_pub_ =
	    parent_->create_publisher<visualization_msgs::msg::MarkerArray>(
	        "debug_markers", 10
	    );

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList PoseIKNavigateToPose::providedPorts() {
	return {
	    BT::InputPort<geometry_msgs::msg::Pose>(
	        "pose", "position relative to base link"
	    ),
	    BT::InputPort<double>("timeout_ms", "Timeout in milliseconds"),
	};
}

BT::NodeStatus PoseIKNavigateToPose::onStart() {
	const auto input_pose = getInput<geometry_msgs::msg::Pose>("pose");
	if (!input_pose) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has no target pose", name().c_str());
		return BT::NodeStatus::FAILURE;
	}
	input_pose_   = input_pose.value();
	commanded_pose_ = input_pose_;

	const auto   timeout_input = getInput<double>("timeout_ms");
	const double timeout_ms = timeout_input ? timeout_input.value() : 10'000.0;
	deadline_ =
	    parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);

	publish_target_marker(visualization_msgs::msg::Marker::ADD);
	publish_target_pose();
	return BT::NodeStatus::RUNNING;
}

void PoseIKNavigateToPose::publish_target_marker(uint8_t action) const {
	visualization_msgs::msg::MarkerArray arr;
	visualization_msgs::msg::Marker      marker{};
	marker.header.frame_id = "base_link";
	marker.header.stamp    = parent_->now();
	marker.ns              = "debug";
	marker.id              = 100;
	marker.action          = action;
	marker.type            = visualization_msgs::msg::Marker::CUBE;
	marker.pose            = input_pose_;
	marker.scale.x         = 0.04;
	marker.scale.y         = 0.04;
	marker.scale.z         = 0.04;
	marker.color.a         = 0.85;
	marker.color.r         = 1.0;
	marker.color.g         = 0.1;
	marker.color.b         = 0.1;
	arr.markers.push_back(marker);
	marker_pub_->publish(arr);
}

void PoseIKNavigateToPose::publish_target_pose() const {
	geometry_msgs::msg::PoseStamped goal{};
	goal.header.frame_id = "base_link";
	goal.header.stamp    = parent_->now();
	goal.pose            = commanded_pose_;
	pose_pub_->publish(goal);
}

BT::NodeStatus PoseIKNavigateToPose::onRunning() {
	const std::string base_frame        = "base_link";
	const std::string end_effector_link = "arm_link_gripper";

	if (parent_->now() >= deadline_) {
		RCLCPP_WARN_STREAM(parent_->get_logger(), name() << " timed out");
		return BT::NodeStatus::FAILURE;
	}

	publish_target_pose();

	try {
		auto base_T_ee = tf_buffer_->lookupTransform(
		    base_frame,
		    end_effector_link,
		    rclcpp::Time(0, 0, parent_->get_clock()->get_clock_type()),
		    rclcpp::Duration::from_seconds(0.1)
		);

		geometry_msgs::msg::Pose identity_pose;
		identity_pose.orientation.w = 1.0;
		const geometry_msgs::msg::Pose current_pose = transformPoseWithTf(
		    identity_pose, toTf(base_T_ee.transform)
		);

		const double dx = commanded_pose_.position.x - current_pose.position.x;
		const double dy = commanded_pose_.position.y - current_pose.position.y;
		const double dz = commanded_pose_.position.z - current_pose.position.z;
		const double linear_error_sq = dx * dx + dy * dy + dz * dz;
		const double angular_error   = quaternionAngleToTarget(
            current_pose.orientation, commanded_pose_.orientation
        );

		if (linear_error_sq < position_tolerance_sq_ &&
		    angular_error < orientation_tolerance_rad_) {
			RCLCPP_INFO_STREAM(
			    parent_->get_logger(),
			    name() << "Returning SUCCESS, linear_error_sq="
			           << linear_error_sq << " angular_error=" << angular_error
			);
			return BT::NodeStatus::SUCCESS;
		}
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    2000,
		    "TF problem: %s",
		    ex.what()
		);
	}

	return BT::NodeStatus::RUNNING;
}

void PoseIKNavigateToPose::onHalted() {
	publish_target_marker(visualization_msgs::msg::Marker::DELETE);
}
