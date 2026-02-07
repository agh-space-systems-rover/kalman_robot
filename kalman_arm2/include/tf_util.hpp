#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace tf_util {

/**
 * Convert a geometry_msgs::msg::Transform to a tf2::Transform.
 */
inline tf2::Transform toTf(const geometry_msgs::msg::Transform &tmsg) {
	tf2::Transform t;
	tf2::fromMsg(tmsg, t);
	return t;
}

/**
 * Convert a geometry_msgs::msg::Pose to a tf2::Transform.
 */
inline tf2::Transform toTf(const geometry_msgs::msg::Pose &pmsg) {
	tf2::Transform t;
	tf2::fromMsg(pmsg, t);
	return t;
}

/**
 * Transform a pose expressed in the child frame to the parent frame.
 *
 * \param in  Pose expressed in child frame.
 * \param T_parent_child  Transform from parent to child.
 * \return Pose expressed in parent frame.
 */
inline geometry_msgs::msg::Pose transformPose(
    const geometry_msgs::msg::Pose &in, const tf2::Transform &T_parent_child
) {
	geometry_msgs::msg::TransformStamped ts;
	ts.transform = tf2::toMsg(T_parent_child);
	geometry_msgs::msg::Pose out;
	tf2::doTransform(in, out, ts);
	return out;
}

} // namespace tf_util
