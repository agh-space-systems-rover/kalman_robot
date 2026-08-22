#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class TransformPose : public BT::SyncActionNode {
  public:
	TransformPose(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();
	BT::NodeStatus       tick() override;

  private:
	rclcpp::Node *parent_;
};
