#include "actions/get_next_goal.hpp"
#include "mission_state.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

GetNextGoal::GetNextGoal(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SyncActionNode(name, config), parent_{parent} {
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	// tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// arm_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	//     "target_twist", 10
	// );
}

BT::PortsList GetNextGoal::providedPorts() {
	return {
	    BT::InputPort<std::shared_ptr<MissionHelper>>("mission_helper"),
	    BT::OutputPort<geometry_msgs::msg::Pose>("next_marker_pose"),
	    BT::OutputPort<uint16_t>("aruco_id")
	};
}

BT::NodeStatus GetNextGoal::tick() {
	const auto mission_helper_opt =
	    getInput<std::shared_ptr<MissionHelper>>("mission_helper");

	if (!mission_helper_opt) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << "Mission helper does not exist"
		);
		return BT::NodeStatus::FAILURE;
	}
	if ((*mission_helper_opt).get() == nullptr) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << "Mission helper is nullptr"
		);
		return BT::NodeStatus::FAILURE;
	}
	const auto &mission_helper = mission_helper_opt.value();

	const auto next_marker_id_opt = mission_helper->get_unvisited_marker();
	if (!next_marker_id_opt.has_value()) {
		RCLCPP_INFO_STREAM(
		    parent_->get_logger(), name() << ": No unvisited markers"
		);

		return BT::NodeStatus::FAILURE;
	}
	const uint16_t next_marker_id = next_marker_id_opt.value();
	const auto     pose           = uv_to_pose(next_marker_id);

	setOutput("next_marker_pose", pose);
	setOutput("aruco_id", next_marker_id);

	RCLCPP_INFO_STREAM(
	    parent_->get_logger(),
	    name() << ": Unvisited marker with id: " << size_t(next_marker_id)
	);
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

geometry_msgs::msg::Pose GetNextGoal::uv_to_pose(size_t ix) const {

	const std::string base_frame       = "base_link";
	const std::string marker_frame     = "model_marker_" + std::to_string(ix);
	auto              T_marker_to_base = tf_buffer_->lookupTransform(
        base_frame,
        marker_frame,
        parent_->now() - std::chrono::milliseconds{100},
        rclcpp::Duration::from_seconds(0.1)
    );
	geometry_msgs::msg::Pose target_pos;
	target_pos.position.z = 0.3; // 30cm
	// target_pos.position.y = -0.1;

	tf2::Transform           T_to_base = toTf(T_marker_to_base.transform);
	geometry_msgs::msg::Pose target_pos_in_base_frame =
	    transformPose_withTf2Transform(target_pos, T_to_base);

	return target_pos_in_base_frame;
}
