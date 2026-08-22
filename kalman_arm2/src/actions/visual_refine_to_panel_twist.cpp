#include "actions/visual_refine_to_panel_twist.hpp"

#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

tf2::Transform poseToTf(const geometry_msgs::msg::Pose &pose) {
	tf2::Transform transform;
	tf2::fromMsg(pose, transform);
	return transform;
}

tf2::Transform transformToTf(
    const geometry_msgs::msg::Transform &transform_msg
) {
	tf2::Transform transform;
	tf2::fromMsg(transform_msg, transform);
	return transform;
}

double rotationAngle(tf2::Quaternion rotation) {
	rotation.normalize();
	if (rotation.w() < 0.0) {
		rotation = tf2::Quaternion(
		    -rotation.x(), -rotation.y(), -rotation.z(), -rotation.w()
		);
	}
	return 2.0 * std::acos(
	                 std::clamp(static_cast<double>(rotation.w()), -1.0, 1.0)
	             );
}

tf2::Vector3 scaledVelocity(
    const tf2::Vector3 &error,
    double              gain,
    double              max_speed,
    double              min_speed,
    double              min_speed_activation_distance
) {
	tf2::Vector3 velocity = error * gain;
	const double speed = velocity.length();
	if (speed > max_speed) {
		velocity *= max_speed / speed;
	} else if (speed > 1e-9 && error.length() > min_speed_activation_distance &&
	           speed < min_speed) {
		velocity *= min_speed / speed;
	}
	return velocity;
}

tf2::Vector3 angularVelocity(
    tf2::Quaternion rotation_error, double gain, double max_speed
) {
	rotation_error.normalize();
	if (rotation_error.w() < 0.0) {
		rotation_error = tf2::Quaternion(
		    -rotation_error.x(),
		    -rotation_error.y(),
		    -rotation_error.z(),
		    -rotation_error.w()
		);
	}

	const double angle = rotationAngle(rotation_error);
	const double sin_half_angle = std::sqrt(
	    std::max(1e-16, 1.0 - rotation_error.w() * rotation_error.w())
	);
	if (angle < 1e-9 || sin_half_angle < 1e-9) {
		return tf2::Vector3(0.0, 0.0, 0.0);
	}

	const tf2::Vector3 axis(
	    rotation_error.x() / sin_half_angle,
	    rotation_error.y() / sin_half_angle,
	    rotation_error.z() / sin_half_angle
	);
	tf2::Vector3 velocity = axis * angle * gain;
	if (velocity.length() > max_speed) {
		velocity *= max_speed / velocity.length();
	}
	return velocity;
}

} // namespace

VisaulRefineToPanelTwist::VisaulRefineToPanelTwist(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
	visual_sub_ = parent_->create_subscription<geometry_msgs::msg::PoseStamped>(
	    "visual_ee_pose",
	    10,
	    std::bind(
	        &VisaulRefineToPanelTwist::visual_pose_callback,
	        this,
	        std::placeholders::_1
	    )
	);
	twist_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);
	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	parent_->get_parameter("visual_refinement_dof", refinement_dof_);
	parent_->get_parameter(
	    "visual_refinement_max_measurement_age_s", max_measurement_age_s_
	);
	parent_->get_parameter(
	    "visual_refinement_position_tolerance", position_tolerance_
	);
	double orientation_tolerance_deg = 3.0;
	parent_->get_parameter(
	    "visual_refinement_orientation_tolerance_deg",
	    orientation_tolerance_deg
	);
	orientation_tolerance_rad_ = orientation_tolerance_deg * M_PI / 180.0;
	parent_->get_parameter("visual_refinement_base_frame", base_frame_);
	parent_->get_parameter("visual_refinement_ee_frame", ee_frame_);
	parent_->get_parameter("visual_refinement_panel_frame", panel_frame_);
	parent_->get_parameter("visual_refinement_twist_linear_kp", linear_kp_);
	parent_->get_parameter(
	    "visual_refinement_twist_max_linear_speed", max_linear_speed_
	);
	parent_->get_parameter(
	    "visual_refinement_twist_min_linear_speed", min_linear_speed_
	);
	parent_->get_parameter(
	    "visual_refinement_twist_min_speed_activation_distance",
	    min_speed_activation_distance_
	);
	parent_->get_parameter("visual_refinement_twist_angular_kp", angular_kp_);
	parent_->get_parameter(
	    "visual_refinement_twist_max_angular_speed", max_angular_speed_
	);
}

BT::PortsList VisaulRefineToPanelTwist::providedPorts() {
	return {
	    BT::InputPort<geometry_msgs::msg::Pose>(
	        "pose", "Literal desired arm_link_end pose in the panel frame"
	    ),
	    BT::InputPort<double>("timeout_ms", "Timeout in milliseconds"),
	};
}

void VisaulRefineToPanelTwist::visual_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
	std::lock_guard<std::mutex> lock(visual_pose_mutex_);
	latest_visual_pose_ = *msg;
}

std::optional<geometry_msgs::msg::PoseStamped>
VisaulRefineToPanelTwist::visual_pose_snapshot() const {
	std::lock_guard<std::mutex> lock(visual_pose_mutex_);
	return latest_visual_pose_;
}

std::optional<tf2::Transform>
VisaulRefineToPanelTwist::current_base_to_ee() const {
	try {
		const auto transform = tf_buffer_->lookupTransform(
		    base_frame_,
		    ee_frame_,
		    tf2::TimePointZero,
		    tf2::durationFromSec(0.1)
		);
		return transformToTf(transform.transform);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    1000,
		    "Visual twist refinement TF lookup failed: %s",
		    ex.what()
		);
		return std::nullopt;
	}
}

void VisaulRefineToPanelTwist::publish_twist(
    const tf2::Vector3 &linear_velocity,
    const tf2::Vector3 &angular_velocity
) const {
	geometry_msgs::msg::TwistStamped twist;
	twist.header.stamp    = parent_->now();
	twist.header.frame_id = base_frame_;
	twist.twist.linear.x  = linear_velocity.x();
	twist.twist.linear.y  = linear_velocity.y();
	twist.twist.linear.z  = linear_velocity.z();
	twist.twist.angular.x = angular_velocity.x();
	twist.twist.angular.y = angular_velocity.y();
	twist.twist.angular.z = angular_velocity.z();
	twist_pub_->publish(twist);
}

void VisaulRefineToPanelTwist::publish_neutral_twist() const {
	publish_twist(
	    tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(0.0, 0.0, 0.0)
	);
}

BT::NodeStatus VisaulRefineToPanelTwist::onStart() {
	publish_neutral_twist();

	const auto desired_pose = getInput<geometry_msgs::msg::Pose>("pose");
	if (!desired_pose) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has no target pose", name().c_str());
		return BT::NodeStatus::FAILURE;
	}
	desired_panel_pose_ = desired_pose.value();
	{
    	tf2::Quaternion tool_alignment;
    	tool_alignment.setRPY(0, M_PI_2, M_PI_2);

    	tf2::Quaternion target_rotation;
    	tf2::fromMsg(desired_panel_pose_.orientation, target_rotation);
    	target_rotation.normalize();

    	const tf2::Quaternion aligned_rotation =
        target_rotation * tool_alignment;

    	desired_panel_pose_.orientation = tf2::toMsg(aligned_rotation);
	}

	const auto timeout = getInput<double>("timeout_ms");
	const double timeout_ms = timeout ? timeout.value() : 15000.0;
	deadline_ =
	    parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);

	if ((refinement_dof_ != 3 && refinement_dof_ != 6) ||
	    !std::isfinite(max_measurement_age_s_) || max_measurement_age_s_ <= 0.0 ||
	    !std::isfinite(position_tolerance_) || position_tolerance_ <= 0.0 ||
	    !std::isfinite(orientation_tolerance_rad_) ||
	    orientation_tolerance_rad_ <= 0.0 || !std::isfinite(linear_kp_) ||
	    linear_kp_ <= 0.0 || !std::isfinite(max_linear_speed_) ||
	    max_linear_speed_ <= 0.0 || !std::isfinite(min_linear_speed_) ||
	    min_linear_speed_ < 0.0 || min_linear_speed_ > max_linear_speed_ ||
	    !std::isfinite(min_speed_activation_distance_) ||
	    min_speed_activation_distance_ < position_tolerance_ ||
	    !std::isfinite(angular_kp_) || angular_kp_ <= 0.0 ||
	    !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0) {
		RCLCPP_ERROR(
		    parent_->get_logger(), "Invalid visual twist refinement parameters"
		);
		return BT::NodeStatus::FAILURE;
	}

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisaulRefineToPanelTwist::onRunning() {
	if (parent_->now() >= deadline_) {
		publish_neutral_twist();
		RCLCPP_ERROR(parent_->get_logger(), "%s timed out", name().c_str());
		return BT::NodeStatus::FAILURE;
	}

	const auto visual_pose = visual_pose_snapshot();
	if (!visual_pose) {
		publish_neutral_twist();
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    2000,
		    "Visual twist refinement is waiting for 'visual_ee_pose'"
		);
		return BT::NodeStatus::RUNNING;
	}

	const double measurement_age_s =
	    (parent_->now() - rclcpp::Time(visual_pose->header.stamp)).seconds();
	if (measurement_age_s < 0.0 || measurement_age_s > max_measurement_age_s_) {
		publish_neutral_twist();
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    2000,
		    "Visual twist refinement measurement age %.3f s is outside [0, %.3f] s",
		    measurement_age_s,
		    max_measurement_age_s_
		);
		return BT::NodeStatus::RUNNING;
	}

	if (visual_pose->header.frame_id != panel_frame_) {
		publish_neutral_twist();
		RCLCPP_ERROR_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    1000,
		    "Visual EE pose uses frame '%s', expected '%s'",
		    visual_pose->header.frame_id.c_str(),
		    panel_frame_.c_str()
		);
		return BT::NodeStatus::RUNNING;
	}

	const tf2::Transform measured_panel_to_ee = poseToTf(visual_pose->pose);
	const tf2::Transform desired_panel_to_ee = poseToTf(desired_panel_pose_);
	const tf2::Transform body_correction =
	    measured_panel_to_ee.inverse() * desired_panel_to_ee;
	const double position_error = body_correction.getOrigin().length();
	const double orientation_error = rotationAngle(body_correction.getRotation());
	RCLCPP_INFO_THROTTLE(
        parent_->get_logger(),
        *parent_->get_clock(),
        500,
        "Visual refinement error: position=%.4f m, rotation=%.4f rad (%.2f deg)",
        position_error,
        orientation_error,
        orientation_error * 180.0 / M_PI
	);
	const bool refine_orientation = refinement_dof_ == 6;

	if (position_error <= position_tolerance_ &&
	    (!refine_orientation ||
	     orientation_error <= orientation_tolerance_rad_)) {
		publish_neutral_twist();
		RCLCPP_INFO(
		    parent_->get_logger(),
		    "Visual twist refinement converged: position=%.4f m, rotation=%.2f deg",
		    position_error,
		    orientation_error * 180.0 / M_PI
		);
		return BT::NodeStatus::SUCCESS;
	}

	const auto base_to_ee = current_base_to_ee();
	if (!base_to_ee) {
		publish_neutral_twist();
		return BT::NodeStatus::RUNNING;
	}

	const tf2::Transform desired_base_to_ee = *base_to_ee * body_correction;
	const tf2::Vector3 linear_error =
	    desired_base_to_ee.getOrigin() - base_to_ee->getOrigin();
	const tf2::Vector3 linear_velocity = scaledVelocity(
	    linear_error,
	    linear_kp_,
	    max_linear_speed_,
	    min_linear_speed_,
	    min_speed_activation_distance_
	);

	tf2::Vector3 angular_velocity(0.0, 0.0, 0.0);
	if (refine_orientation) {
		const tf2::Quaternion spatial_rotation_error =
		    desired_base_to_ee.getRotation() *
		    base_to_ee->getRotation().inverse();
		angular_velocity = angularVelocity(
		    spatial_rotation_error, angular_kp_, max_angular_speed_
		);
	}

	publish_twist(linear_velocity, angular_velocity);
	return BT::NodeStatus::RUNNING;
}

void VisaulRefineToPanelTwist::onHalted() {
	publish_neutral_twist();
}
