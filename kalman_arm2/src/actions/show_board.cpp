#include "actions/show_board.hpp"
#include "mission_state.hpp"
// #include "transform_ema_filter.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ShowBoard::ShowBoard(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {

	RCLCPP_ERROR_STREAM(
	    parent_->get_logger(),
	    this->name() << " ShowBoard constructor is being run..."
	);
	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	static_broadcaster_ =
	    std::make_shared<tf2_ros::StaticTransformBroadcaster>(parent_);

	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(parent_);

	marker_pub_ =
	    parent_->create_publisher<visualization_msgs::msg::MarkerArray>(
	        "debug_markers", 10
	    );
}

BT::PortsList ShowBoard::providedPorts() {
	return {
	    BT::InputPort<std::shared_ptr<MissionHelper>>("mission_helper"),
	};
}

BT::NodeStatus ShowBoard::onStart() {
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
	const PanelLayout &layout = mission_helper->state.layout_;

	visualization_msgs::msg::MarkerArray arr;
	rclcpp::Time                         stamp = parent_->now();

	constexpr float MARKER_SIZE = 0.06;
	{ // Board as a whole
		visualization_msgs::msg::Marker m{};
		m.header.frame_id    = "uv_board";
		m.header.stamp       = stamp;
		m.ns                 = "debug";
		m.id                 = 0;
		m.action             = visualization_msgs::msg::Marker::ADD;
		m.type               = visualization_msgs::msg::Marker::CUBE;
		m.scale.x            = layout.board_width;
		m.scale.y            = layout.board_height;
		m.scale.z            = 0.01;
		m.pose.orientation.w = 1.0;
		m.pose.position.x = layout.board_width / 2.0F - MARKER_SIZE;
		m.pose.position.y = -layout.board_height / 2.0F + MARKER_SIZE;
		m.color.a = 0.15;
		m.color.r = 0.1;
		m.color.g = 0.4;
		m.color.b = 1.0;
		arr.markers.push_back(m);
	}
	int i {1};
	for (const auto &[_, marker_info] : layout.markers) {
		visualization_msgs::msg::Marker m{};
		m.header.frame_id    = "uv_board";
		m.header.stamp       = stamp;
		m.ns                 = "debug";
		m.id                 = i++;
		m.action             = visualization_msgs::msg::Marker::ADD;
		m.type               = visualization_msgs::msg::Marker::CUBE;
		m.scale.x            = MARKER_SIZE;
		m.scale.y            = MARKER_SIZE;
		m.scale.z            = MARKER_SIZE;
		m.pose.orientation.w = 1.0;
		m.pose.position.x = marker_info.u;
		m.pose.position.y = -marker_info.v;
		m.color.a = 0.15;
		m.color.r = 0.1;
		m.color.g = 0.4;
		m.color.b = 1.0;
		m.text = std::to_string(marker_info.id);
		arr.markers.push_back(m);
	}
	marker_pub_->publish(arr);

	return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ShowBoard::onRunning() {
	return BT::NodeStatus::SUCCESS;
}

void ShowBoard::onHalted() {}
