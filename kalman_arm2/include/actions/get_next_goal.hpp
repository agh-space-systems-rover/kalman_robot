#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

class GetNextGoal : public BT::SyncActionNode {
  public:
	GetNextGoal(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus tick() override;

  private:
	rclcpp::Node *parent_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;

	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	geometry_msgs::msg::Pose uv_to_pose(size_t ix) const;

	// void move_arm_to_pose(const geometry_msgs::msg::Pose &pose);
};
