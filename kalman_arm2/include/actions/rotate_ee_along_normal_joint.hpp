#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>

class RotateEEAlongNormalJoint : public BT::StatefulActionNode {
  public:
	RotateEEAlongNormalJoint(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	void current_position_callback(
	    const kalman_interfaces::msg::ArmValues::SharedPtr msg
	);
	std::optional<double> current_joint_6_position() const;
	void publish_joint_6_velocity(double velocity) const;

	rclcpp::Node *parent_;
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    current_position_sub_;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr velocity_pub_;

	mutable std::mutex    position_mutex_;
	std::optional<double> current_joint_6_position_;
	std::optional<double> requested_angle_rad_;
	std::optional<double> target_position_;
	rclcpp::Time          deadline_;
	double                joint_kp_{1.5};
	double                max_joint_speed_{0.3};
	double                min_joint_speed_{0.05};
	double                tolerance_rad_{0.0174532925};
};
