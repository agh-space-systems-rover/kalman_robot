#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
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

using Matrix6 = std::array<double, 36>;

Matrix6 covariance_jacobian(
  const tf2::Matrix3x3 & output_from_input,
  const tf2::Vector3 & child_offset_in_input)
{
  Matrix6 jacobian{};
  const double skew[3][3] = {
    {0.0, -child_offset_in_input.z(), child_offset_in_input.y()},
    {child_offset_in_input.z(), 0.0, -child_offset_in_input.x()},
    {-child_offset_in_input.y(), child_offset_in_input.x(), 0.0},
  };

  for (size_t row = 0; row < 3; ++row) {
    for (size_t column = 0; column < 3; ++column) {
      const double rotation = output_from_input[row][column];
      jacobian[row * 6 + column] = rotation;
      jacobian[(row + 3) * 6 + column + 3] = rotation;

      // p_child = p_input + R * r. Orientation uncertainty therefore
      // contributes -R_frame * [r]_x * dtheta to position uncertainty.
      double coupling = 0.0;
      for (size_t index = 0; index < 3; ++index) {
        coupling -= output_from_input[row][index] * skew[index][column];
      }
      jacobian[row * 6 + column + 3] = coupling;
    }
  }
  return jacobian;
}

Matrix6 transform_covariance(const Matrix6 & input, const Matrix6 & jacobian)
{
  Matrix6 sanitized{};
  for (size_t index = 0; index < input.size(); ++index) {
    sanitized[index] = std::isfinite(input[index]) ? input[index] : 0.0;
  }

  Matrix6 intermediate{};
  Matrix6 output{};
  for (size_t row = 0; row < 6; ++row) {
    for (size_t column = 0; column < 6; ++column) {
      for (size_t index = 0; index < 6; ++index) {
        intermediate[row * 6 + column] +=
          jacobian[row * 6 + index] * sanitized[index * 6 + column];
      }
    }
  }
  for (size_t row = 0; row < 6; ++row) {
    for (size_t column = 0; column < 6; ++column) {
      for (size_t index = 0; index < 6; ++index) {
        output[row * 6 + column] +=
          intermediate[row * 6 + index] * jacobian[column * 6 + index];
      }
    }
  }

  // Numerical noise and imperfect upstream matrices can break exact
  // symmetry, while consumers expect a symmetric covariance.
  for (size_t row = 0; row < 6; ++row) {
    for (size_t column = row + 1; column < 6; ++column) {
      const double symmetric =
        0.5 * (output[row * 6 + column] + output[column * 6 + row]);
      output[row * 6 + column] = symmetric;
      output[column * 6 + row] = symmetric;
    }
  }
  return output;
}

void apply_variance_floors(
  Matrix6 & covariance,
  const double linear_floor,
  const double angular_floor)
{
  for (size_t index = 0; index < 3; ++index) {
    covariance[index * 6 + index] =
      std::max(covariance[index * 6 + index], linear_floor);
    covariance[(index + 3) * 6 + index + 3] =
      std::max(covariance[(index + 3) * 6 + index + 3], angular_floor);
  }
}

class OdometryFrameTransformer : public rclcpp::Node
{
public:
  OdometryFrameTransformer()
  : Node("odometry_frame_transformer"),
    parent_frame_(declare_parameter<std::string>("parent_frame", "odom")),
    child_frame_(declare_parameter<std::string>("child_frame", "base_link")),
    pose_position_variance_floor_(
      declare_parameter<double>("pose_position_variance_floor", 0.0)),
    pose_orientation_variance_floor_(
      declare_parameter<double>("pose_orientation_variance_floor", 0.0)),
    twist_linear_variance_floor_(
      declare_parameter<double>("twist_linear_variance_floor", 0.0)),
    twist_angular_variance_floor_(
      declare_parameter<double>("twist_angular_variance_floor", 0.0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (
      pose_position_variance_floor_ < 0.0 ||
      pose_orientation_variance_floor_ < 0.0 ||
      twist_linear_variance_floor_ < 0.0 ||
      twist_angular_variance_floor_ < 0.0)
    {
      throw std::invalid_argument("Covariance variance floors must be non-negative");
    }
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

      // Transform pose covariance from camera_init/body to odom/base_link.
      // The base_link lever arm couples orientation uncertainty into position.
      const tf2::Vector3 child_offset = raw_child_to_child.getOrigin();
      const tf2::Vector3 child_offset_in_source =
        source_to_raw_child.getBasis() * child_offset;
      const auto pose_jacobian = covariance_jacobian(
        parent_to_source.getBasis(), child_offset_in_source);
      output.pose.covariance = transform_covariance(
        msg->pose.covariance, pose_jacobian);
      apply_variance_floors(
        output.pose.covariance,
        pose_position_variance_floor_,
        pose_orientation_variance_floor_);

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
      const tf2::Matrix3x3 child_from_raw =
        raw_child_to_child.getBasis().transpose();
      const tf2::Vector3 linear_child =
        child_from_raw * (linear_raw + angular_raw.cross(child_offset));
      const tf2::Vector3 angular_child = child_from_raw * angular_raw;

      output.twist.twist.linear = tf2::toMsg(linear_child);
      output.twist.twist.angular = tf2::toMsg(angular_child);
      const auto twist_jacobian =
        covariance_jacobian(child_from_raw, child_offset);
      output.twist.covariance = transform_covariance(
        msg->twist.covariance, twist_jacobian);
      apply_variance_floors(
        output.twist.covariance,
        twist_linear_variance_floor_,
        twist_angular_variance_floor_);
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
  double pose_position_variance_floor_;
  double pose_orientation_variance_floor_;
  double twist_linear_variance_floor_;
  double twist_angular_variance_floor_;
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
