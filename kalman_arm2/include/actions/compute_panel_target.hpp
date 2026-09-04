#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ComputePanelTarget : public BT::SyncActionNode {
  public:
    ComputePanelTarget(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node *parent
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

  private:
    geometry_msgs::msg::Pose resolve_panel_relative_pose() const;

    rclcpp::Node *parent_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
