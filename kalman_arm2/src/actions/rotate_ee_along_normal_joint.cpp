#include "actions/rotate_ee_along_normal_joint.hpp"

#include <algorithm>
#include <cmath>

RotateEEAlongNormalJoint::RotateEEAlongNormalJoint(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
	current_position_sub_ =
	    parent_->create_subscription<kalman_interfaces::msg::ArmValues>(
	        "current_pos",
	        10,
	        std::bind(
	            &RotateEEAlongNormalJoint::current_position_callback,
	            this,
	            std::placeholders::_1
	        )
	    );
	velocity_pub_ = parent_->create_publisher<kalman_interfaces::msg::ArmValues>(
	    "target_vel/joints", 10
	);

	parent_->get_parameter("rotate_ee_joint_kp", joint_kp_);
	parent_->get_parameter("rotate_ee_joint_max_speed", max_joint_speed_);
	parent_->get_parameter("rotate_ee_joint_min_speed", min_joint_speed_);
	double tolerance_deg = 1.0;
	parent_->get_parameter("rotate_ee_tolerance_deg", tolerance_deg);
	tolerance_rad_ = tolerance_deg * M_PI / 180.0;
}

BT::PortsList RotateEEAlongNormalJoint::providedPorts() {
	return {
	    BT::InputPort<double>("degrees", "Signed relative arm_joint_6 rotation"),
	    BT::InputPort<double>("timeout_ms", 10000.0, "Timeout in milliseconds"),
	};
}

void RotateEEAlongNormalJoint::current_position_callback(
    const kalman_interfaces::msg::ArmValues::SharedPtr msg
) {
	std::lock_guard<std::mutex> lock(position_mutex_);
	current_joint_6_position_ = msg->joints[5];
}

std::optional<double>
RotateEEAlongNormalJoint::current_joint_6_position() const {
	std::lock_guard<std::mutex> lock(position_mutex_);
	return current_joint_6_position_;
}

void RotateEEAlongNormalJoint::publish_joint_6_velocity(double velocity) const {
	kalman_interfaces::msg::ArmValues command;
	command.header.stamp = parent_->now();
	command.joints.fill(0.0);
	command.joints[5] = velocity;
	command.jaw       = 0.0;
	velocity_pub_->publish(command);
}

BT::NodeStatus RotateEEAlongNormalJoint::onStart() {
	publish_joint_6_velocity(0.0);
	target_position_.reset();

	const auto degrees_input = getInput<double>("degrees");
	const double timeout_ms = getInput<double>("timeout_ms").value_or(10000.0);
	if (!degrees_input || !std::isfinite(degrees_input.value()) ||
	    std::abs(degrees_input.value()) > 180.0 || !std::isfinite(timeout_ms) ||
	    timeout_ms <= 0.0 || !std::isfinite(joint_kp_) || joint_kp_ <= 0.0 ||
	    !std::isfinite(max_joint_speed_) || max_joint_speed_ <= 0.0 ||
	    !std::isfinite(min_joint_speed_) || min_joint_speed_ < 0.0 ||
	    min_joint_speed_ > max_joint_speed_ ||
	    !std::isfinite(tolerance_rad_) || tolerance_rad_ <= 0.0) {
		RCLCPP_ERROR(
		    parent_->get_logger(),
		    "%s requires finite degrees in [-180, 180] and valid joint parameters",
		    name().c_str()
		);
		return BT::NodeStatus::FAILURE;
	}

	requested_angle_rad_ = degrees_input.value() * M_PI / 180.0;
	deadline_ =
	    parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateEEAlongNormalJoint::onRunning() {
	if (parent_->now() >= deadline_) {
		publish_joint_6_velocity(0.0);
		RCLCPP_ERROR(parent_->get_logger(), "%s timed out", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	const auto current_position = current_joint_6_position();
	if (!current_position || !std::isfinite(*current_position)) {
		publish_joint_6_velocity(0.0);
		return BT::NodeStatus::RUNNING;
	}
	if (!target_position_) {
		target_position_ = *current_position + *requested_angle_rad_;
	}

	const double error = *target_position_ - *current_position;
	if (std::abs(error) <= tolerance_rad_) {
		publish_joint_6_velocity(0.0);
		RCLCPP_INFO(
		    parent_->get_logger(),
		    "%s reached target with %.2f deg joint error",
		    name().c_str(),
		    std::abs(error) * 180.0 / M_PI
		);
		return BT::NodeStatus::SUCCESS;
	}

	double velocity = std::clamp(
	    joint_kp_ * error, -max_joint_speed_, max_joint_speed_
	);
	if (std::abs(velocity) < min_joint_speed_) {
		velocity = std::copysign(min_joint_speed_, velocity);
	}
	publish_joint_6_velocity(velocity);
	return BT::NodeStatus::RUNNING;
}

void RotateEEAlongNormalJoint::onHalted() {
	publish_joint_6_velocity(0.0);
}
