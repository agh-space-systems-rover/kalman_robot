#include "actions/gripper_actions.hpp"

#include <cmath>

GripperCommandAction::GripperCommandAction(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent,
    double                       target_position
)
    : BT::StatefulActionNode(name, config), parent_(parent),
      target_position_(target_position) {
	current_position_sub_ =
	    parent_->create_subscription<kalman_interfaces::msg::ArmValues>(
	        "current_pos",
	        10,
	        std::bind(
	            &GripperCommandAction::current_position_callback,
	            this,
	            std::placeholders::_1
	        )
	    );
	target_pub_ = parent_->create_publisher<kalman_interfaces::msg::ArmValues>(
	    "target_pos/jaw", 10
	);
}

BT::PortsList GripperCommandAction::commonPorts() {
	return {
	    BT::InputPort<double>("timeout_ms", 5000.0, "Timeout in milliseconds"),
	    BT::InputPort<double>("tolerance", 0.05, "Jaw position tolerance in radians"),
	};
}

void GripperCommandAction::current_position_callback(
    const kalman_interfaces::msg::ArmValues::SharedPtr msg
) {
	std::lock_guard<std::mutex> lock(position_mutex_);
	current_position_ = msg->jaw;
}

std::optional<double> GripperCommandAction::current_position() const {
	std::lock_guard<std::mutex> lock(position_mutex_);
	return current_position_;
}

void GripperCommandAction::publish_target(double position) const {
	kalman_interfaces::msg::ArmValues target;
	target.header.stamp = parent_->now();
	target.jaw          = position;
	target_pub_->publish(target);
}

BT::NodeStatus GripperCommandAction::onStart() {
	const double timeout_ms = getInput<double>("timeout_ms").value_or(5000.0);
	tolerance_ = getInput<double>("tolerance").value_or(0.05);
	if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0 ||
	    !std::isfinite(tolerance_) || tolerance_ <= 0.0) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has invalid parameters", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	deadline_ =
	    parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
	publish_target(target_position_);
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperCommandAction::onRunning() {
	const auto position = current_position();
	if (position && std::isfinite(*position) &&
	    std::abs(*position - target_position_) <= tolerance_) {
		RCLCPP_INFO(
		    parent_->get_logger(),
		    "%s reached jaw position %.3f",
		    name().c_str(),
		    *position
		);
		return BT::NodeStatus::SUCCESS;
	}

	if (parent_->now() >= deadline_) {
		if (position && std::isfinite(*position)) {
			publish_target(*position);
		}
		RCLCPP_ERROR(
		    parent_->get_logger(),
		    "%s timed out waiting for jaw position %.3f",
		    name().c_str(),
		    target_position_
		);
		return BT::NodeStatus::FAILURE;
	}

	publish_target(target_position_);
	return BT::NodeStatus::RUNNING;
}

void GripperCommandAction::onHalted() {
	const auto position = current_position();
	if (position && std::isfinite(*position)) {
		publish_target(*position);
	}
}

OpenGripper::OpenGripper(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : GripperCommandAction(
          name,
          config,
          parent,
          parent->get_parameter("gripper_open_position").as_double()
      ) {}

BT::PortsList OpenGripper::providedPorts() {
	return commonPorts();
}

CloseGripper::CloseGripper(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : GripperCommandAction(
          name,
          config,
          parent,
          parent->get_parameter("gripper_closed_position").as_double()
      ) {}

BT::PortsList CloseGripper::providedPorts() {
	return commonPorts();
}
