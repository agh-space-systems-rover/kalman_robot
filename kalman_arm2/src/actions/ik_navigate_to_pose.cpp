#include "actions/ik_navigate_to_pose.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <bitset>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/action/detail/arm_goto_joint_pose__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_client.hpp>
#include <string>
#include <tf2/LinearMath/QuadWord.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Scalar.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

IKNavigateToPose::IKNavigateToPose(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {
	arm_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(parent_);
}

BT::PortsList IKNavigateToPose::providedPorts() {
	return {BT::InputPort<geometry_msgs::msg::Pose>(
	    "pose", "position relative to base link"
	)};
}

BT::NodeStatus IKNavigateToPose::onStart() {
	pose = getInput<geometry_msgs::msg::Pose>("pose").value();
	return BT::NodeStatus::RUNNING;
}

namespace {
static tf2::Transform toTf(const geometry_msgs::msg::Transform &tmsg) {
	tf2::Transform t;
	tf2::fromMsg(tmsg, t);
	return t;
}
static tf2::Transform toTf(const geometry_msgs::msg::Pose &pmsg) {
	tf2::Transform t;
	tf2::fromMsg(pmsg, t);
	return t;
}
static geometry_msgs::msg::Pose transformPose_withTf2Transform(
    const geometry_msgs::msg::Pose &in,
    const tf2::Transform           &T_parent_child
) // transform from parent->child
{
	geometry_msgs::msg::TransformStamped ts;
	ts.transform = tf2::toMsg(T_parent_child); // fill only the transform part

	geometry_msgs::msg::Pose out;
	tf2::doTransform(in, out, ts); // applies both rotation & translation
	return out;                    // pose now expressed in 'parent' frame
}

inline geometry_msgs::msg::Vector3 angularVelToTarget(
    const geometry_msgs::msg::Quaternion& q_current_msg,
    const geometry_msgs::msg::Quaternion& q_target_msg,
    double kp = 2.0,                 // tune this
    double max_w = 2.0               // rad/s (limit)
){
  // Convert & normalize
  tf2::Quaternion q_c, q_t;
  tf2::fromMsg(q_current_msg, q_c);
  tf2::fromMsg(q_target_msg,  q_t);
  q_c.normalize();
  q_t.normalize();

  // Quaternion error (body frame): R_err = R_c^T * R_t
  // -> rotate from current to target, expressed in the current/body frame
  tf2::Quaternion q_err = q_c.inverse() * q_t;
//   tf2::Quaternion q_err = q_c * q_t;
  q_err.normalize();

  // Ensure shortest path (quaternions double-cover SO(3))
  if (q_err.getW() < 0.0) {
    q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
  }

  // Axis-angle from q_err
  double w   = std::clamp(static_cast<double>(q_err.getW()), -1.0, 1.0);
  double ang = 2.0 * std::acos(w);                 // in [0, pi]
  double s   = std::sqrt(std::max(1e-16, 1.0 - w*w)); // = sin(ang/2)

  tf2::Vector3 axis(1.0, 0.0, 0.0);               // arbitrary when angle ~ 0
  if (s > 1e-8) {
    axis = tf2::Vector3(q_err.getX()/s, q_err.getY()/s, q_err.getZ()/s);
  }
  tf2::Vector3 rotvec = axis * ang;               // axis * angle

  // Proportional angular velocity (optional: add -Kd*ω_meas for PD)
  tf2::Vector3 w_cmd = kp * rotvec;

  // Limit magnitude
  double n = w_cmd.length();
  if (n > max_w) {
    w_cmd *= (max_w / n);
  }

  geometry_msgs::msg::Vector3 out;
  out.x = w_cmd.x();
  out.y = w_cmd.y();
  out.z = w_cmd.z();
  return out;
}

} // namespace

BT::NodeStatus IKNavigateToPose::onRunning() {
	const std::string base_frame_        = "base_link";
	const std::string end_effector_frame = "arm_link_end";

	try {
		// 1) base -> camera (from robot tree, at detection time)
		auto base_T_cam_st = tf_buffer_->lookupTransform(
		    base_frame_,
		    end_effector_frame,
		    parent_->now() - std::chrono::milliseconds{100},
		    rclcpp::Duration::from_seconds(0.1)
		);

		tf2::Transform T_base_cam = toTf(base_T_cam_st.transform);
		const geometry_msgs::msg::Pose zero_pose;
		const geometry_msgs::msg::Pose current_pose =
		    transformPose_withTf2Transform(zero_pose, T_base_cam);

		{
			geometry_msgs::msg::TwistStamped twist{};

			twist.header.frame_id = base_frame_;

			const float factor = -0.1;

			twist.twist.linear.x =
			    (current_pose.position.x - pose.position.x) * factor;
			twist.twist.linear.y =
			    (current_pose.position.y - pose.position.y) * factor;
			twist.twist.linear.z =
			    (current_pose.position.z - pose.position.z) * factor;

			{
				// We want the end effector to be perpendicular to marker
				tf2::Quaternion q;
				q.setRPY(0, M_PI_2, M_PI_2);
				auto pose_copy = pose;
				tf2::Quaternion target_rotation;
				tf2::fromMsg(pose.orientation, target_rotation);

				const auto rot = target_rotation * q;
				pose_copy.orientation = tf2::toMsg(rot);

				// geometry_msgs::msg::TransformStamped transform;
				// transform.header.frame_id = "base_link";
				// transform.child_frame_id = "Test_debug_frame";
				// transform.transform.rotation = pose_copy.orientation;
				// transform.transform.translation.x = pose_copy.position.x;
				// transform.transform.translation.y = pose_copy.position.y;
				// transform.transform.translation.z = pose_copy.position.z;

				// static_broadcaster_->sendTransform(transform);

				twist.twist.angular = angularVelToTarget(current_pose.orientation, pose_copy.orientation);
			}

			const auto v = twist.twist.linear;

			const auto vec3mag = [](geometry_msgs::msg::Vector3 v) -> float {
				return v.x * v.x + v.y * v.y + v.z * v.z;
			};
			const float magnitude = vec3mag(v);
			if (magnitude < 1e-4) {
				RCLCPP_INFO_STREAM(
				    parent_->get_logger(), name() << "Returning SUCCESS"
				);
				return BT::NodeStatus::SUCCESS;
			}

			arm_pub_->publish(twist);
		}

	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    2000,
		    "TF problem: %s",
		    ex.what()
		);
	}

	return BT::NodeStatus::RUNNING;
}
void IKNavigateToPose::onHalted() {}
