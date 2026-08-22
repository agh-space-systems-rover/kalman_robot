#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class VisaulRefineToPanelTwist : public BT::StatefulActionNode {
  public:
	VisaulRefineToPanelTwist(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	void visual_pose_callback(
	    const geometry_msgs::msg::PoseStamped::SharedPtr msg
	);
	std::optional<geometry_msgs::msg::PoseStamped>
	visual_pose_snapshot() const;
	std::optional<tf2::Transform> current_base_to_ee() const;
	void publish_neutral_twist() const;
	void publish_twist(
	    const tf2::Vector3 &linear_velocity,
	    const tf2::Vector3 &angular_velocity
	) const;

	rclcpp::Node *parent_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr visual_sub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::optional<geometry_msgs::msg::PoseStamped> latest_visual_pose_;
	mutable std::mutex visual_pose_mutex_;
	geometry_msgs::msg::Pose desired_panel_pose_;
	rclcpp::Time             deadline_;

	std::string base_frame_;
	std::string ee_frame_;
	std::string panel_frame_;
	int         refinement_dof_{3};
	double      max_measurement_age_s_{0.5};
	double      position_tolerance_{0.01};
	double      orientation_tolerance_rad_{0.0523598776};
	double      linear_kp_{0.8};
	double      max_linear_speed_{0.12};
	double      min_linear_speed_{0.015};
	double      min_speed_activation_distance_{0.03};
	double      angular_kp_{2.0};
	double      max_angular_speed_{0.6};
};
