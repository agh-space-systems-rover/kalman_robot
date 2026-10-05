#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Node to convert camera_init frame postion on /Odometry topic into a acceptable position for EKF node
namespace kalman_slam
{

class OdometryFrameTransformer : public rclcpp::Node
{
public:
  OdometryFrameTransformer()
  : Node("odometry_frame_transformer"),
    parent_frame_(declare_parameter<std::string>("parent_frame", "odom")),
    child_frame_(declare_parameter<std::string>("child_frame", "base_link")),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry/transformed", 20);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/raw", 20,
      std::bind(&OdometryFrameTransformer::odometry_callback, this, std::placeholders::_1));
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    try {
      const auto parent_to_source_msg = tf_buffer_.lookupTransform(
        parent_frame_, msg->header.frame_id, tf2::TimePointZero);
      const auto raw_child_to_child_msg = tf_buffer_.lookupTransform(
        msg->child_frame_id, child_frame_, tf2::TimePointZero);

      tf2::Transform parent_to_source;
      tf2::Transform source_to_raw_child;
      tf2::Transform raw_child_to_child;
      tf2::fromMsg(parent_to_source_msg.transform, parent_to_source);
      tf2::fromMsg(msg->pose.pose, source_to_raw_child);
      tf2::fromMsg(raw_child_to_child_msg.transform, raw_child_to_child);

      auto output = *msg;
      output.header.frame_id = parent_frame_;
      output.child_frame_id = child_frame_;
      const auto transformed_pose = tf2::toMsg(
        parent_to_source * source_to_raw_child * raw_child_to_child);
      output.pose.pose.position.x = transformed_pose.translation.x;
      output.pose.pose.position.y = transformed_pose.translation.y;
      output.pose.pose.position.z = transformed_pose.translation.z;
      output.pose.pose.orientation = transformed_pose.rotation;

      // nav_msgs/Odometry twist is expressed in child_frame_id. Shift the
      // velocity from the raw IMU origin to base_link, then rotate its axes.
      const tf2::Vector3 linear_raw(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
      const tf2::Vector3 angular_raw(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z);
      const tf2::Vector3 child_offset = raw_child_to_child.getOrigin();
      const tf2::Matrix3x3 child_from_raw =
        raw_child_to_child.getBasis().transpose();
      const tf2::Vector3 linear_child =
        child_from_raw * (linear_raw + angular_raw.cross(child_offset));
      const tf2::Vector3 angular_child = child_from_raw * angular_raw;

      output.twist.twist.linear = tf2::toMsg(linear_child);
      output.twist.twist.angular = tf2::toMsg(angular_child);
      publisher_->publish(output);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform FAST-LIO odometry to %s -> %s: %s",
        parent_frame_.c_str(), child_frame_.c_str(), exception.what());
    }
  }

  std::string parent_frame_;
  std::string child_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace kalman_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kalman_slam::OdometryFrameTransformer>());
  rclcpp::shutdown();
  return 0;
}
