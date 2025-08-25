#include "actions/build_uv.hpp"
#include "mission_state.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BuildUV::BuildUV(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	static_broadcaster_ =
	    std::make_shared<tf2_ros::StaticTransformBroadcaster>(parent_);

	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(parent_);
}

BT::PortsList BuildUV::providedPorts() {
	return {
	    BT::InputPort<std::shared_ptr<MissionHelper>>("mission_helper"),
	};
}

BT::NodeStatus BuildUV::onStart() {
	// Make sure there exists a mission helper
	auto mission_helper_opt =
	    getInput<std::shared_ptr<MissionHelper>>("mission_helper");
	if (!mission_helper_opt.has_value()) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << ": mission helper not set"
		);
		return BT::NodeStatus::FAILURE;
	}
	auto mission_helper = mission_helper_opt.value();
	if (nullptr == mission_helper) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << ": mission helper is nullptr"
		);
		return BT::NodeStatus::FAILURE;
	}
	const auto& markers = mission_helper->state.layout_.markers;

	std::vector<UVAnchor> anchors = { };

	for(const auto&[id, marker_info] : markers){
		UVAnchor anchor;
		anchor.tf_name = "marker_" + std::to_string(id) + "_avg"; // TODO: make this a helper in state, to ensure consistency accross nodes
		anchor.uv = {marker_info.u, -marker_info.v};
		anchors.emplace_back(std::move(anchor));
	}

	Eigen::Isometry3d T_base_to_board{};
	if (!buildUnitScale(anchors, *tf_buffer_, "base_link", "uv_board", T_base_to_board)) {
		return BT::NodeStatus::FAILURE;
	}

	geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = parent_->now();
    ts.header.frame_id = "base_link";
    ts.child_frame_id  = "uv_board";
    ts.transform = tf2::eigenToTransform(T_base_to_board).transform;
	static_broadcaster_->sendTransform(ts);

	return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BuildUV::onRunning() {
	return BT::NodeStatus::SUCCESS;
}

void BuildUV::onHalted() {}
