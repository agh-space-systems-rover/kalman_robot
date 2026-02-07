#pragma once
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_state.hpp"

class HasNextGoal : public BT::SimpleConditionNode
{
public:
  HasNextGoal(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node* parent);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node* parent_{};
  // MissionHelper* mission_helper{};
};
