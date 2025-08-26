#include "actions/wait_for_uv_request.hpp"
#include "mission_state.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

WaitForUVRequest::WaitForUVRequest(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	uv_sub_ = parent_->create_subscription<geometry_msgs::msg::Point>(
	    "uv_point",
	    1,
	    std::bind(
	        &WaitForUVRequest::uv_point_callback, this, std::placeholders::_1
	    )
	);
}

BT::PortsList WaitForUVRequest::providedPorts() {
	return {
	    BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
	};
}

BT::NodeStatus WaitForUVRequest::onStart() {
	point_opt_ = {};
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForUVRequest::onRunning() {
	if (!point_opt_.has_value()){
		return BT::NodeStatus::RUNNING; // just wait
	}
	const auto& point = point_opt_.value();

	const auto     pose           = uv_to_pose(point);

	setOutput("target_pose", pose);

	return BT::NodeStatus::SUCCESS;
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

} // namespace

geometry_msgs::msg::Pose WaitForUVRequest::uv_to_pose(geometry_msgs::msg::Point point) const {

	const std::string base_frame       = "base_link";
	const std::string marker_frame     = "uv_board";
	auto              T_marker_to_base = tf_buffer_->lookupTransform(
        base_frame,
        marker_frame,
        parent_->now() - std::chrono::milliseconds{100},
        rclcpp::Duration::from_seconds(0.1)
    );
	geometry_msgs::msg::Pose target_pos;
	target_pos.position.x = point.x;
	target_pos.position.y = -point.y;
	target_pos.position.z = point.z + 0.05; // 5cm

	tf2::Transform           T_to_base = toTf(T_marker_to_base.transform);
	geometry_msgs::msg::Pose target_pos_in_base_frame =
	    transformPose_withTf2Transform(target_pos, T_to_base);

	return target_pos_in_base_frame;
}

void WaitForUVRequest::uv_point_callback(
    geometry_msgs::msg::Point::SharedPtr msg
) {
	point_opt_ = *msg;
}
