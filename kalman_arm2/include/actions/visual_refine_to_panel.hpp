#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class VisualRefineToPanel : public BT::StatefulActionNode {
  public:
    VisualRefineToPanel(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node *parent
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    enum class State { WAITING_FOR_MEASUREMENT, MOVING, SETTLING };

    void visual_pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    );
    void publish_commanded_pose() const;
    void hold_current_pose();
    std::optional<geometry_msgs::msg::PoseStamped> visual_pose_snapshot() const;
    std::optional<tf2::Transform> current_base_to_ee() const;
    const char *state_name() const;
    BT::NodeStatus begin_correction();
    bool nominal_target_reached() const;

    rclcpp::Node *parent_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr visual_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::optional<geometry_msgs::msg::PoseStamped> latest_visual_pose_;
    mutable std::mutex visual_pose_mutex_;
    geometry_msgs::msg::Pose desired_panel_pose_;
    tf2::Transform commanded_base_to_ee_;
    rclcpp::Time deadline_;
    rclcpp::Time settle_deadline_;
    rclcpp::Time last_used_measurement_stamp_;
    State state_{State::WAITING_FOR_MEASUREMENT};
    int correction_count_{0};

    std::string base_frame_;
    std::string ee_frame_;
    std::string panel_frame_;
    int refinement_dof_{3};
    int max_corrections_{10};
    double max_measurement_age_s_{0.3};
    double settle_time_s_{0.4};
    double max_translation_step_{0.03};
    double max_rotation_step_rad_{0.0872664626};
    double visual_position_tolerance_{0.01};
    double visual_orientation_tolerance_rad_{0.0523598776};
    double nominal_position_tolerance_{0.01};
    double nominal_orientation_tolerance_rad_{0.10};
};
