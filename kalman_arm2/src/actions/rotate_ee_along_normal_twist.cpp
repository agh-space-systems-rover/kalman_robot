#include "actions/rotate_ee_along_normal_twist.hpp"

#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

tf2::Transform transformToTf(
    const geometry_msgs::msg::Transform &transform_msg
) {
	tf2::Transform transform;
	tf2::fromMsg(transform_msg, transform);
	return transform;
}

double rotationAngle(tf2::Quaternion rotation) {
	rotation.normalize();
	if (rotation.w() < 0.0) {
		rotation = tf2::Quaternion(
		    -rotation.x(), -rotation.y(), -rotation.z(), -rotation.w()
		);
	}
	return 2.0 * std::acos(
	                 std::clamp(static_cast<double>(rotation.w()), -1.0, 1.0)
	             );
}

tf2::Vector3 rotationVector(tf2::Quaternion rotation) {
	rotation.normalize();
	if (rotation.w() < 0.0) {
		rotation = tf2::Quaternion(
		    -rotation.x(), -rotation.y(), -rotation.z(), -rotation.w()
		);
	}
	const double angle = rotationAngle(rotation);
	const double sin_half_angle = std::sqrt(
	    std::max(1e-16, 1.0 - rotation.w() * rotation.w())
	);
	if (angle < 1e-9 || sin_half_angle < 1e-9) {
		return tf2::Vector3(0.0, 0.0, 0.0);
	}
	return tf2::Vector3(
	           rotation.x() / sin_half_angle,
	           rotation.y() / sin_half_angle,
	           rotation.z() / sin_half_angle
	       ) *
	       angle;
}

} // namespace

RotateEEAlongNormalTwist::RotateEEAlongNormalTwist(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
	twist_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);
	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	parent_->get_parameter("rotate_ee_base_frame", base_frame_);
	parent_->get_parameter("rotate_ee_frame", ee_frame_);
	parent_->get_parameter("rotate_ee_angular_kp", angular_kp_);
	parent_->get_parameter("rotate_ee_max_angular_speed", max_angular_speed_);
	parent_->get_parameter("rotate_ee_min_angular_speed", min_angular_speed_);
	double tolerance_deg = 1.0;
	parent_->get_parameter("rotate_ee_tolerance_deg", tolerance_deg);
	tolerance_rad_ = tolerance_deg * M_PI / 180.0;
}

BT::PortsList RotateEEAlongNormalTwist::providedPorts() {
	return {
	    BT::InputPort<double>("degrees", "Signed rotation around EE-local +X"),
	    BT::InputPort<double>("timeout_ms", 10000.0, "Timeout in milliseconds"),
	};
}

std::optional<tf2::Transform> RotateEEAlongNormalTwist::current_base_to_ee() const {
	try {
		const auto transform = tf_buffer_->lookupTransform(
		    base_frame_,
		    ee_frame_,
		    tf2::TimePointZero,
		    tf2::durationFromSec(0.1)
		);
		return transformToTf(transform.transform);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    1000,
		    "RotateEEAlongNormalTwist TF lookup failed: %s",
		    ex.what()
		);
		return std::nullopt;
	}
}

void RotateEEAlongNormalTwist::publish_angular_velocity(
    const tf2::Vector3 &velocity
) const {
	geometry_msgs::msg::TwistStamped twist;
	twist.header.stamp     = parent_->now();
	twist.header.frame_id  = base_frame_;
	twist.twist.angular.x  = velocity.x();
	twist.twist.angular.y  = velocity.y();
	twist.twist.angular.z  = velocity.z();
	twist_pub_->publish(twist);
}

void RotateEEAlongNormalTwist::publish_neutral_twist() const {
	publish_angular_velocity(tf2::Vector3(0.0, 0.0, 0.0));
}

BT::NodeStatus RotateEEAlongNormalTwist::onStart() {
	publish_neutral_twist();

	const auto degrees_input = getInput<double>("degrees");
	const double timeout_ms = getInput<double>("timeout_ms").value_or(10000.0);
	if (!degrees_input || !std::isfinite(degrees_input.value()) ||
	    std::abs(degrees_input.value()) > 180.0 || !std::isfinite(timeout_ms) ||
	    timeout_ms <= 0.0 || !std::isfinite(angular_kp_) || angular_kp_ <= 0.0 ||
	    !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0 ||
	    !std::isfinite(min_angular_speed_) || min_angular_speed_ < 0.0 ||
	    min_angular_speed_ > max_angular_speed_ ||
	    !std::isfinite(tolerance_rad_) || tolerance_rad_ <= 0.0) {
		RCLCPP_ERROR(
		    parent_->get_logger(),
		    "%s requires finite degrees in [-180, 180] and valid rotation parameters",
		    name().c_str()
		);
		return BT::NodeStatus::FAILURE;
	}

	const auto base_to_ee = current_base_to_ee();
	if (!base_to_ee) {
		return BT::NodeStatus::FAILURE;
	}

	const double angle_rad = degrees_input.value() * M_PI / 180.0;
	tf2::Quaternion relative_rotation;
	relative_rotation.setRotation(tf2::Vector3(1.0, 0.0, 0.0), angle_rad);
	target_rotation_ = base_to_ee->getRotation() * relative_rotation;
	target_rotation_.normalize();
	deadline_ =
	    parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateEEAlongNormalTwist::onRunning() {
	if (parent_->now() >= deadline_) {
		publish_neutral_twist();
		RCLCPP_ERROR(parent_->get_logger(), "%s timed out", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	const auto base_to_ee = current_base_to_ee();
	if (!base_to_ee) {
		publish_neutral_twist();
		return BT::NodeStatus::RUNNING;
	}

	tf2::Quaternion rotation_error =
	    target_rotation_ * base_to_ee->getRotation().inverse();
	rotation_error.normalize();
	const double error_angle = rotationAngle(rotation_error);
	if (error_angle <= tolerance_rad_) {
		publish_neutral_twist();
		RCLCPP_INFO(
		    parent_->get_logger(),
		    "%s reached target with %.2f deg angular error",
		    name().c_str(),
		    error_angle * 180.0 / M_PI
		);
		return BT::NodeStatus::SUCCESS;
	}

	tf2::Vector3 angular_velocity = rotationVector(rotation_error) * angular_kp_;
	const double speed = angular_velocity.length();
	if (speed > max_angular_speed_) {
		angular_velocity *= max_angular_speed_ / speed;
	} else if (speed > 1e-9 && speed < min_angular_speed_) {
		angular_velocity *= min_angular_speed_ / speed;
	}
	publish_angular_velocity(angular_velocity);
	return BT::NodeStatus::RUNNING;
}

void RotateEEAlongNormalTwist::onHalted() {
	publish_neutral_twist();
}
