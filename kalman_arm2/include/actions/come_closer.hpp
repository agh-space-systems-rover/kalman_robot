#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <cstdint>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <sys/types.h>
#include <thread>
#include <iostream>

class ComeCloser : public BT::StatefulActionNode
{
public:
  ComeCloser(
	  const std::string           &name,
	  const BT::NodeConfiguration &config,
	  rclcpp::Node                *parent
  );

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void
  aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);

  rclcpp::Node *parent_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;

  aruco_opencv_msgs::msg::ArucoDetection last_aruco_msg;
  constexpr static uint16_t MARKER_INV = std::numeric_limits<uint16_t>::max();
  uint16_t tracked_marker = MARKER_INV;
  std::optional<geometry_msgs::msg::PoseStamped> last_aruco_pose{};

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;
};
