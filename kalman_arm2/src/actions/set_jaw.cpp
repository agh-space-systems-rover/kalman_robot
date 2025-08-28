#include "actions/set_jaw.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <bitset>
#include <chrono>
#include <cstdlib>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

SetJaw::SetJaw(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {
	pub_ = parent_->create_publisher<kalman_interfaces::msg::ArmValues>(
	    "target_pos/jaw", 10
	);

	using namespace std::placeholders;
	sub_ = parent_->create_subscription<kalman_interfaces::msg::ArmValues>(
	    "current_pos",
	    10,
	    std::bind(&SetJaw::arm_callback, this, std::placeholders::_1)
	);
}

BT::PortsList SetJaw::providedPorts() {
	return {BT::InputPort<double>("jaw", "Gripper jaw position")};
}

BT::NodeStatus SetJaw::onStart() {
	const auto jaw_opt = getInput<double>("jaw");
	if (!jaw_opt.has_value()) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << " jaw angle not provided!"
		);
		return BT::NodeStatus::FAILURE;
	}
	target_jaw_angle  = jaw_opt.value();
	current_jaw_angle = {};
	start_time_       = parent_->now();

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetJaw::onRunning() {
	if (!current_jaw_angle.has_value()) {
		RCLCPP_INFO_STREAM(
		    parent_->get_logger(), name() << " waiting for jaw value"
		);
		return BT::NodeStatus::RUNNING;
	}
	const double current = current_jaw_angle.value();
	const double target  = target_jaw_angle;
	const double dx      = std::abs(current - target);
	if (dx < 0.01) {
		return BT::NodeStatus::SUCCESS;
	}
	kalman_interfaces::msg::ArmValues arm_vals;
	arm_vals.jaw = target;
	pub_->publish(arm_vals);

	const auto time_since_start = parent_->now() - start_time_;
	if (time_since_start > std::chrono::seconds{2}) {
		RCLCPP_INFO_STREAM(
		    parent_->get_logger(),
		    name() << ": arm did not close withing 2 seconds. Fuck waiting and "
		              "return true"
		);
		return BT::NodeStatus::SUCCESS;
	}

	return BT::NodeStatus::RUNNING;
}

void SetJaw::onHalted() {}

void SetJaw::arm_callback(
    const kalman_interfaces::msg::ArmValues::SharedPtr msg
) {
	current_jaw_angle = msg->jaw;
}
