#include "actions/transform_pose.hpp"

#include <cmath>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TransformPose::TransformPose(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SyncActionNode(name, config), parent_(parent) {}

BT::PortsList TransformPose::providedPorts() {
	return {
	    BT::InputPort<geometry_msgs::msg::Pose>("pose", "Input pose"),
	    BT::InputPort<double>("x", 0.0, "Local X translation in meters"),
	    BT::InputPort<double>("y", 0.0, "Local Y translation in meters"),
	    BT::InputPort<double>("z", 0.0, "Local Z translation in meters"),
	    BT::OutputPort<geometry_msgs::msg::Pose>(
	        "transformed_pose", "Pose after applying local translation"
	    ),
	};
}

BT::NodeStatus TransformPose::tick() {
	const auto input_pose = getInput<geometry_msgs::msg::Pose>("pose");
	if (!input_pose) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has no input pose", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	const double x = getInput<double>("x").value_or(0.0);
	const double y = getInput<double>("y").value_or(0.0);
	const double z = getInput<double>("z").value_or(0.0);
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
		RCLCPP_ERROR(
		    parent_->get_logger(), "%s received non-finite translation", name().c_str()
		);
		return BT::NodeStatus::FAILURE;
	}

	const auto &orientation = input_pose->orientation;
	const double quaternion_norm_squared =
	    orientation.x * orientation.x + orientation.y * orientation.y +
	    orientation.z * orientation.z + orientation.w * orientation.w;
	if (!std::isfinite(quaternion_norm_squared) ||
	    quaternion_norm_squared <= 1e-12) {
		RCLCPP_ERROR(
		    parent_->get_logger(), "%s received invalid orientation", name().c_str()
		);
		return BT::NodeStatus::FAILURE;
	}

	tf2::Transform input_transform;
	tf2::fromMsg(input_pose.value(), input_transform);
	input_transform.getRotation().normalize();

	tf2::Transform translation;
	translation.setIdentity();
	translation.setOrigin(tf2::Vector3(x, y, z));

	const tf2::Transform output_transform = input_transform * translation;
	geometry_msgs::msg::Pose output_pose;
	output_pose.position.x = output_transform.getOrigin().x();
	output_pose.position.y = output_transform.getOrigin().y();
	output_pose.position.z = output_transform.getOrigin().z();
	output_pose.orientation = tf2::toMsg(output_transform.getRotation());
	setOutput("transformed_pose", output_pose);
	return BT::NodeStatus::SUCCESS;
}
