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
	    BT::InputPort<double>("rx_deg", 0.0, "Local X rotation in degrees"),
	    BT::InputPort<double>("ry_deg", 0.0, "Local Y rotation in degrees"),
	    BT::InputPort<double>("rz_deg", 0.0, "Local Z rotation in degrees"),
	    BT::OutputPort<geometry_msgs::msg::Pose>(
	        "transformed_pose", "Pose after applying the local transform"
	    ),
	};
}

BT::NodeStatus TransformPose::tick() {
	const auto input_pose = getInput<geometry_msgs::msg::Pose>("pose");
	if (!input_pose) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has no input pose", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	const double x      = getInput<double>("x").value_or(0.0);
	const double y      = getInput<double>("y").value_or(0.0);
	const double z      = getInput<double>("z").value_or(0.0);
	const double rx_deg = getInput<double>("rx_deg").value_or(0.0);
	const double ry_deg = getInput<double>("ry_deg").value_or(0.0);
	const double rz_deg = getInput<double>("rz_deg").value_or(0.0);
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
	    !std::isfinite(rx_deg) || !std::isfinite(ry_deg) ||
	    !std::isfinite(rz_deg)) {
		RCLCPP_ERROR(
		    parent_->get_logger(),
		    "%s received a non-finite transform component",
		    name().c_str()
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

	constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
	tf2::Quaternion relative_rotation;
	relative_rotation.setRPY(
	    rx_deg * degrees_to_radians,
	    ry_deg * degrees_to_radians,
	    rz_deg * degrees_to_radians
	);
	relative_rotation.normalize();

	tf2::Transform relative_transform;
	relative_transform.setOrigin(tf2::Vector3(x, y, z));
	relative_transform.setRotation(relative_rotation);

	const tf2::Transform output_transform = input_transform * relative_transform;
	geometry_msgs::msg::Pose output_pose;
	output_pose.position.x = output_transform.getOrigin().x();
	output_pose.position.y = output_transform.getOrigin().y();
	output_pose.position.z = output_transform.getOrigin().z();
	output_pose.orientation = tf2::toMsg(output_transform.getRotation());
	setOutput("transformed_pose", output_pose);
	return BT::NodeStatus::SUCCESS;
}
