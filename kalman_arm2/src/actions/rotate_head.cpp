#include "actions/rotate_head.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

RotateHead::RotateHead(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {

	arm_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);
}

BT::PortsList RotateHead::providedPorts() {
	return {
	    BT::InputPort<double>("rx"),
	    BT::InputPort<double>("ry"),
	    BT::InputPort<double>("rz")
	};
}

BT::NodeStatus RotateHead::onStart() {
	twist_stamped.header.frame_id = "arm_link_end";
	auto &angular = twist_stamped.twist.angular;
	angular.x = getInput<double>("rx").value_or(0.0);
	angular.y = getInput<double>("ry").value_or(0.0);
	angular.z = getInput<double>("rz").value_or(0.0);
	if (angular == geometry_msgs::msg::Vector3{}){
		return BT::NodeStatus::SUCCESS;
	}
	start_time = parent_->now();

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateHead::onRunning() {
	// FIXME: make timeout a parameter
	if (parent_->now() - start_time > std::chrono::seconds{3}) {
		onHalted();
		return BT::NodeStatus::SUCCESS;
	}
	RCLCPP_INFO(parent_->get_logger(), "Rotating head");
	arm_pub_->publish(twist_stamped);
	return BT::NodeStatus::RUNNING;
}

void RotateHead::onHalted() {
	geometry_msgs::msg::TwistStamped zero_twist{};
	arm_pub_->publish(zero_twist);
}
