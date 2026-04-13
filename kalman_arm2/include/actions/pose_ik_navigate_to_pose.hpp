#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

class PoseIKNavigateToPose : public BT::StatefulActionNode {
  public:
	PoseIKNavigateToPose(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	rclcpp::Node *parent_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
	                                            marker_pub_;
	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	geometry_msgs::msg::Pose                    input_pose_;
	geometry_msgs::msg::Pose                    commanded_pose_;
	rclcpp::Time                                deadline_;
	double                                      position_tolerance_sq_{1e-4};
	double                                      orientation_tolerance_rad_{0.15};

	void publish_target_marker(uint8_t action) const;
	void publish_target_pose() const;
};
