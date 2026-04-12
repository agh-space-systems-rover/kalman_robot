#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

class IsPanelPoseGood : public BT::SimpleConditionNode {
  public:
    IsPanelPoseGood(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node *parent
    );

    static BT::PortsList providedPorts();

  private:
    BT::NodeStatus tick() override;
    void panel_pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    );

    bool pose_is_finite(const geometry_msgs::msg::PoseStamped &pose) const;

    rclcpp::Node *parent_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr panel_sub_;
    std::optional<geometry_msgs::msg::PoseStamped> last_panel_pose_;
};
