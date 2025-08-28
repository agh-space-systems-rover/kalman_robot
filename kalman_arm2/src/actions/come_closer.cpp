#include "actions/come_closer.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

ComeCloser::ComeCloser(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {

	aruco_sub_ =
	    parent_->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
	        "/d435_arm/aruco_detections", // topic name
	        10,                           // queue size
	        std::bind(&ComeCloser::aruco_callback, this, std::placeholders::_1)
	    );

	arm_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);
}

BT::PortsList ComeCloser::providedPorts() {
	return {
	    BT::OutputPort<geometry_msgs::msg::PoseStamped>("aruco_position"),
	    BT::InputPort<uint16_t>("aruco_id")
	};
}

BT::NodeStatus ComeCloser::onStart() {
	const auto aruco_id_opt = getInput<uint16_t>("aruco_id");
	if (!aruco_id_opt.has_value()) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << ": aruco_id is empty, failing"
		);
		return BT::NodeStatus::FAILURE;
	}
	tracked_marker  = aruco_id_opt.value();
	last_aruco_pose = {};

	RCLCPP_INFO_STREAM(
	    parent_->get_logger(),
	    name() << ": setting tracked marker to: " << tracked_marker
	);
	start_time = parent_->now();
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComeCloser::onRunning() {
	geometry_msgs::msg::TwistStamped twist{};
	if (!last_aruco_pose.has_value()) {
		RCLCPP_ERROR_STREAM_THROTTLE(
		    parent_->get_logger(),
			*parent_->get_clock(),
			1000,
		    name() << ": last_aruco_pose is empty, waiting for a message"
		);
		if (parent_->now() - start_time > std::chrono::seconds{5}){
			return BT::NodeStatus::FAILURE;
		}
		
		return BT::NodeStatus::RUNNING;
	}

	const geometry_msgs::msg::Pose &mp = last_aruco_pose->pose;
	twist.header                       = last_aruco_pose->header;

	// FIXME: Using time for collision detection is stupid and naive
	constexpr std::chrono::milliseconds MAX_ARUCO_DETECTION_AGE{1000}; // Using higher number for simulation
	const auto time_since_last_msg = parent_->now() - last_aruco_pose->header.stamp;
	if (time_since_last_msg > MAX_ARUCO_DETECTION_AGE){
		geometry_msgs::msg::TwistStamped zero_vel{};
		zero_vel.header = last_aruco_pose->header;
		arm_pub_->publish(zero_vel);

		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(),
		    name() << ": did not get an aruco detection during the last " << MAX_ARUCO_DETECTION_AGE.count() << "ms "
		);
		return BT::NodeStatus::FAILURE;
	}

	twist.twist.linear.x = mp.position.x;
	twist.twist.linear.y = mp.position.y;
	twist.twist.linear.z = mp.position.z - 0.3;

	const geometry_msgs::msg::Vector3 linear = twist.twist.linear;
	const auto vec3mag = [](geometry_msgs::msg::Vector3 v) -> float {
		return v.x * v.x + v.y * v.y + v.z * v.z;
	};
	const float magnitude = vec3mag(linear);

	RCLCPP_INFO_THROTTLE(parent_->get_logger(), *parent_->get_clock(), 1000, "Movement magnitude²: %f", magnitude);

	if (magnitude < 1e-3) {
		// Stop
		geometry_msgs::msg::TwistStamped zero_vel{};
		zero_vel.header = last_aruco_pose->header;
		arm_pub_->publish(zero_vel);

		return BT::NodeStatus::SUCCESS;
	}

	arm_pub_->publish(twist);

	return BT::NodeStatus::RUNNING;
}

void ComeCloser::onHalted() {
	tracked_marker  = MARKER_INV;
	last_aruco_pose = {};
}

void ComeCloser::aruco_callback(
    const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg
) {
	if (msg->markers.empty()) {
		return;
	}
	if (tracked_marker == MARKER_INV) {
		last_aruco_msg = *msg;
	} else {
		const auto &markers = msg->markers;

		const auto is_currently_selected_marker =
		    [&](const aruco_opencv_msgs::msg::MarkerPose &mp) -> bool {
			return mp.marker_id == tracked_marker;
		};
		const auto found_it = std::find_if(
		    std::begin(markers), std::end(markers), is_currently_selected_marker
		);
		std::string marker_ids_str = "";
		for (const auto &m : markers) {
			marker_ids_str += " " + std::to_string(size_t(m.marker_id));
		}
		if (found_it == std::end(markers)) {
			RCLCPP_DEBUG_STREAM(
			    parent_->get_logger(),
			    name() << " Got marker message, but no marker has expected ID: "
			           << size_t(tracked_marker) << ", got only"
			           << marker_ids_str
			);
			return;
		}

		const auto                      my_marker = *found_it;
		geometry_msgs::msg::PoseStamped pose;
		pose.pose       = my_marker.pose;
		pose.header     = msg->header;
		last_aruco_pose = pose;
	}
}
