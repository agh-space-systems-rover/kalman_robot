#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>

class GripperCommandAction : public BT::StatefulActionNode {
  public:
	GripperCommandAction(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent,
	    double                       target_position
	);

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  protected:
	static BT::PortsList commonPorts();
	BT::NodeStatus start_command(double timeout_ms, double tolerance);
	void set_target_position(double target_position);
	rclcpp::Node *parent_;

  private:
	void current_position_callback(
	    const kalman_interfaces::msg::ArmValues::SharedPtr msg
	);
	void publish_target(double position) const;
	std::optional<double> current_position() const;

	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    current_position_sub_;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr target_pub_;

	mutable std::mutex    position_mutex_;
	std::optional<double> current_position_;
	double                target_position_;
	double                tolerance_{0.05};
	rclcpp::Time          deadline_;
};

class OpenGripper : public GripperCommandAction {
  public:
	OpenGripper(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();
};

class CloseGripper : public GripperCommandAction {
  public:
	CloseGripper(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();
};

class SetGripperAngle : public GripperCommandAction {
  public:
	SetGripperAngle(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();
	BT::NodeStatus       onStart() override;
};
