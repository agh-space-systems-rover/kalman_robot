#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <cstdint>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <sys/types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

class IKNavigateToPose : public BT::StatefulActionNode {
  public:
	IKNavigateToPose(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	rclcpp::Node                                                  *parent_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;
	std::unique_ptr<tf2_ros::Buffer>                               tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener>                    tf_listener_;
  geometry_msgs::msg::Pose pose;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};
