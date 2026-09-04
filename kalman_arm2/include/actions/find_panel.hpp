#pragma once

#include "mission_state.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class FindPanel : public BT::SyncActionNode {
  public:
    FindPanel(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node *parent
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

  private:
    void panel_pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
    );
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg
    );

    std::shared_ptr<MissionHelper> mission_helper() const;
    geometry_msgs::msg::PoseWithCovarianceStamped latest_panel_pose_in_base(
        const std::string &base_frame
    ) const;
    geometry_msgs::msg::Pose compute_target_pose(
        const geometry_msgs::msg::Pose &base_panel_pose
    ) const;

    rclcpp::Node *parent_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        panel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_panel_pose_;
    std::optional<sensor_msgs::msg::CameraInfo> last_camera_info_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
