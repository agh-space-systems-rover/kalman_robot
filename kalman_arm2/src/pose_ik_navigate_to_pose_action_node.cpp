#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kalman_interfaces/action/pose_ik_navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace kalman_arm2 {
namespace {
double quaternionAngle(
    const geometry_msgs::msg::Quaternion &current_message,
    const geometry_msgs::msg::Quaternion &target_message
) {
	tf2::Quaternion current;
	tf2::Quaternion target;
	tf2::fromMsg(current_message, current);
	tf2::fromMsg(target_message, target);
	current.normalize();
	target.normalize();

	tf2::Quaternion error = current.inverse() * target;
	error.normalize();
	const double absolute_w = std::abs(static_cast<double>(error.getW()));
	return 2.0 * std::acos(std::clamp(absolute_w, 0.0, 1.0));
}
} // namespace

class PoseIKNavigateToPoseAction : public rclcpp::Node {
  public:
	using Action = kalman_interfaces::action::PoseIKNavigateToPose;
	using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

	explicit PoseIKNavigateToPoseAction(const rclcpp::NodeOptions &options)
	    : Node("pose_ik_navigate_to_pose_action", options) {
		position_tolerance_ =
		    declare_parameter<double>("position_tolerance", 0.01);
		orientation_tolerance_ =
		    declare_parameter<double>("orientation_tolerance_rad", 0.15);
		default_timeout_seconds_ =
		    declare_parameter<double>("default_timeout_seconds", 10.0);
		const double update_rate = declare_parameter<double>("update_rate", 20.0);

		target_publisher_ =
		    create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
		marker_publisher_ =
		    create_publisher<visualization_msgs::msg::Marker>("debug_marker", 10);
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener_ =
		    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		using namespace std::placeholders;
		action_server_ = rclcpp_action::create_server<Action>(
		    this,
		    "pose_ik_navigate_to_pose",
		    std::bind(&PoseIKNavigateToPoseAction::handleGoal, this, _1, _2),
		    std::bind(&PoseIKNavigateToPoseAction::handleCancel, this, _1),
		    std::bind(&PoseIKNavigateToPoseAction::handleAccepted, this, _1)
		);

		const auto period = std::chrono::duration<double>(
		    1.0 / std::max(update_rate, 1e-3)
		);
		timer_ = create_wall_timer(
		    period, std::bind(&PoseIKNavigateToPoseAction::tick, this)
		);
	}

  private:
	rclcpp_action::GoalResponse handleGoal(
	    const rclcpp_action::GoalUUID &,
	    const std::shared_ptr<const Action::Goal> goal
	) {
		std::lock_guard<std::mutex> lock(mutex_);
		if (current_goal_ && current_goal_->is_active()) {
			RCLCPP_WARN(get_logger(), "Rejecting pose IK goal: server is busy");
			return rclcpp_action::GoalResponse::REJECT;
		}
		if (goal->base_frame.empty() || goal->end_effector_link.empty()) {
			RCLCPP_WARN(get_logger(), "Rejecting pose IK goal with empty frame name");
			return rclcpp_action::GoalResponse::REJECT;
		}
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handleCancel(
	    const std::shared_ptr<GoalHandle>
	) {
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
		std::lock_guard<std::mutex> lock(mutex_);
		current_goal_ = goal_handle;
		const double requested_timeout = goal_handle->get_goal()->timeout_seconds;
		const double timeout = requested_timeout > 0.0
		                           ? requested_timeout
		                           : default_timeout_seconds_;
		deadline_ = now() + rclcpp::Duration::from_seconds(timeout);
		publishMarker(visualization_msgs::msg::Marker::ADD);
		publishTarget();
	}

	void tick() {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!current_goal_ || !current_goal_->is_active()) {
			return;
		}

		if (current_goal_->is_canceling()) {
			auto result = std::make_shared<Action::Result>();
			result->result = false;
			result->message = "Navigation canceled";
			current_goal_->canceled(result);
			finishGoal();
			return;
		}

		if (now() >= deadline_) {
			auto result = std::make_shared<Action::Result>();
			result->result = false;
			result->message = "Navigation timed out";
			current_goal_->abort(result);
			finishGoal();
			return;
		}

		publishTarget();
		const auto &goal = *current_goal_->get_goal();
		try {
			const auto transform = tf_buffer_->lookupTransform(
			    goal.base_frame,
			    goal.end_effector_link,
			    rclcpp::Time(0, 0, get_clock()->get_clock_type()),
			    rclcpp::Duration::from_seconds(0.1)
			);

			const double dx = goal.pose.position.x - transform.transform.translation.x;
			const double dy = goal.pose.position.y - transform.transform.translation.y;
			const double dz = goal.pose.position.z - transform.transform.translation.z;
			const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
			const double orientation_error = quaternionAngle(
			    transform.transform.rotation, goal.pose.orientation
			);

			auto feedback = std::make_shared<Action::Feedback>();
			feedback->position_error = position_error;
			feedback->orientation_error = orientation_error;
			current_goal_->publish_feedback(feedback);

			if (position_error <= position_tolerance_ &&
			    orientation_error <= orientation_tolerance_) {
				auto result = std::make_shared<Action::Result>();
				result->result = true;
				result->message = "Target pose reached";
				current_goal_->succeed(result);
				finishGoal();
			}
		} catch (const tf2::TransformException &error) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    2000,
			    "Waiting for transform %s -> %s: %s",
			    goal.base_frame.c_str(),
			    goal.end_effector_link.c_str(),
			    error.what()
			);
		}
	}

	void publishTarget() const {
		const auto &goal = *current_goal_->get_goal();
		geometry_msgs::msg::PoseStamped target;
		target.header.stamp = now();
		target.header.frame_id = goal.base_frame;
		target.pose = goal.pose;
		target_publisher_->publish(target);
	}

	void publishMarker(int action) const {
		if (!current_goal_) {
			return;
		}
		const auto &goal = *current_goal_->get_goal();
		visualization_msgs::msg::Marker marker;
		marker.header.stamp = now();
		marker.header.frame_id = goal.base_frame;
		marker.ns = "pose_ik_target";
		marker.id = 100;
		marker.type = visualization_msgs::msg::Marker::CUBE;
		marker.action = action;
		marker.pose = goal.pose;
		marker.scale.x = 0.04;
		marker.scale.y = 0.04;
		marker.scale.z = 0.04;
		marker.color.a = 0.85;
		marker.color.r = 1.0;
		marker.color.g = 0.1;
		marker.color.b = 0.1;
		marker_publisher_->publish(marker);
	}

	void finishGoal() {
		publishMarker(visualization_msgs::msg::Marker::DELETE);
		current_goal_.reset();
	}

	double position_tolerance_;
	double orientation_tolerance_;
	double default_timeout_seconds_;
	std::mutex mutex_;
	rclcpp::Time deadline_;
	std::shared_ptr<GoalHandle> current_goal_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
	    target_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
	    marker_publisher_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	rclcpp_action::Server<Action>::SharedPtr action_server_;
	rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PoseIKNavigateToPoseAction)
