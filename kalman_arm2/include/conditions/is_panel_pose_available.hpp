#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

class IsPanelPoseAvailable : public BT::SimpleConditionNode {
  public:
    IsPanelPoseAvailable(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node *parent
    );

    static BT::PortsList providedPorts();

  private:
    BT::NodeStatus tick() override;
    void panel_pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
    );

    rclcpp::Node *parent_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        panel_sub_;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_panel_pose_;
};
