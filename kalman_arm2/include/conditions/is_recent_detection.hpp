#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

class IsRecentDetection : public BT::SimpleConditionNode
{
public:
  IsRecentDetection(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node* parent);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus check(void);

  BT::NodeStatus tick() override;

  void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);

private:
  using ArucoDetection = aruco_opencv_msgs::msg::ArucoDetection;

  rclcpp::Node* parent_;
  rclcpp::Subscription<ArucoDetection>::SharedPtr aruco_sub_;

  rclcpp::Time last_detection_{parent_->now() - std::chrono::seconds{50000}};

  ArucoDetection last_msg_;

  aruco_opencv_msgs::msg::MarkerPose get_first_aruco_position() const;
};
