#include "conditions/is_recent_detection.hpp"
#include <aruco_opencv_msgs/msg/detail/marker_pose__struct.hpp>
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>

IsRecentDetection::IsRecentDetection(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SimpleConditionNode(
          name,
          std::bind(&IsRecentDetection::check, this),
          config
      ),
      parent_(parent) {

	aruco_sub_ = parent_->create_subscription<ArucoDetection>(
	    "/d435_arm/aruco_detections", // topic name
	    10,                           // queue size
	    std::bind(&IsRecentDetection::aruco_callback, this, std::placeholders::_1)
	);
}

void IsRecentDetection::aruco_callback(
    const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg
) {
  if (msg->markers.empty()){
    return;
  }
  // FIXME: store this detection, in check() just compare the header and current time
  last_detection_ = parent_->now();
  last_msg_ = *msg;
}

BT::NodeStatus IsRecentDetection::check(void) {
  BT::Optional<float> max_age_msec_optional = getInput<float>("max_age_ms");
  if (!max_age_msec_optional) {
    RCLCPP_ERROR_STREAM(
      parent_->get_logger(),
      name() << ": max_age_ms not set, check file: ");  // TODO: Add a filename here
    return BT::NodeStatus::FAILURE;
  }


  const float max_age_msec = max_age_msec_optional.value();
  const auto max_age_seconds = rclcpp::Duration::from_seconds(max_age_msec / 1000.0F);

  const rclcpp::Duration last_message_age = parent_->now() - last_detection_;
  if (last_message_age < max_age_seconds){
    RCLCPP_INFO_STREAM(parent_->get_logger(), name() << "Returning SUCCESS");

    const auto first_marker_detection = get_first_aruco_position();
    const auto first_marker_id = first_marker_detection.marker_id;
    geometry_msgs::msg::PoseStamped first_marker_position{};
    first_marker_position.pose = first_marker_detection.pose;
    first_marker_position.header = last_msg_.header;

    setOutput("first_marker_position", first_marker_position);
    setOutput("first_marker_id", first_marker_id);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsRecentDetection::providedPorts() {
	return {
	    BT::InputPort<float>("max_age_ms", "How old can last message be?"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("first_marker_position"),
      BT::OutputPort<uint16_t>("first_marker_id")
	};
}

BT::NodeStatus IsRecentDetection::tick() {
	return check();
}

aruco_opencv_msgs::msg::MarkerPose IsRecentDetection::get_first_aruco_position() const {
	if (last_msg_.markers.empty()) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(),
		    name() << "Last marker is empty, this should never happen!"
		);
	}
  return last_msg_.markers.at(0);
}

