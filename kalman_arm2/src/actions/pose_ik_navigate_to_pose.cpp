#include "actions/pose_ik_navigate_to_pose.hpp"

#include <chrono>
#include <string>

#include <rclcpp_action/create_client.hpp>

PoseIKNavigateToPose::PoseIKNavigateToPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
	client_ = rclcpp_action::create_client<Action>(
	    parent_->get_node_base_interface(),
	    parent_->get_node_graph_interface(),
	    parent_->get_node_logging_interface(),
	    parent_->get_node_waitables_interface(),
	    "pose_ik_navigate_to_pose"
	);
}

BT::PortsList PoseIKNavigateToPose::providedPorts() {
	return {
	    BT::InputPort<geometry_msgs::msg::Pose>(
	        "pose", "Target pose expressed in base_frame"
	    ),
	    BT::InputPort<std::string>("base_frame", "base_link", "Base frame"),
	    BT::InputPort<std::string>(
	        "end_effector_link", "arm_link_gripper", "End-effector TF link"
	    ),
	    BT::InputPort<double>("timeout_ms", 10'000.0, "Timeout in milliseconds"),
	};
}

BT::NodeStatus PoseIKNavigateToPose::onStart() {
	const auto pose = getInput<geometry_msgs::msg::Pose>("pose");
	if (!pose) {
		RCLCPP_ERROR(parent_->get_logger(), "%s has no target pose", name().c_str());
		return BT::NodeStatus::FAILURE;
	}
	if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
		RCLCPP_ERROR(
		    parent_->get_logger(),
		    "%s: pose IK navigation action server is unavailable",
		    name().c_str()
		);
		return BT::NodeStatus::FAILURE;
	}

	Action::Goal goal;
	goal.pose = pose.value();
	goal.base_frame = getInput<std::string>("base_frame").value();
	goal.end_effector_link =
	    getInput<std::string>("end_effector_link").value();
	goal.timeout_seconds = getInput<double>("timeout_ms").value() / 1000.0;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		last_result_.reset();
		last_message_.clear();
	}
	halted_ = false;

	auto options = rclcpp_action::Client<Action>::SendGoalOptions{};
	options.result_callback = [this](const GoalHandle::WrappedResult &result) {
		std::lock_guard<std::mutex> lock(mutex_);
		last_result_ = result.code;
		if (result.result) {
			last_message_ = result.result->message;
		}
	};
	goal_handle_future_ = client_->async_send_goal(goal, options);
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PoseIKNavigateToPose::onRunning() {
	if (halted_) {
		return BT::NodeStatus::FAILURE;
	}

	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (last_result_) {
			if (*last_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
				return BT::NodeStatus::SUCCESS;
			}
			RCLCPP_ERROR(
			    parent_->get_logger(),
			    "%s failed: %s",
			    name().c_str(),
			    last_message_.c_str()
			);
			return BT::NodeStatus::FAILURE;
		}
	}

	if (goal_handle_future_.valid() &&
	    goal_handle_future_.wait_for(std::chrono::milliseconds(0)) ==
	        std::future_status::ready) {
		const auto goal_handle = goal_handle_future_.get();
		if (!goal_handle) {
			RCLCPP_ERROR(
			    parent_->get_logger(), "%s action goal was rejected", name().c_str()
			);
			return BT::NodeStatus::FAILURE;
		}
	}

	return BT::NodeStatus::RUNNING;
}

void PoseIKNavigateToPose::onHalted() {
	halted_ = true;
	if (goal_handle_future_.valid() &&
	    goal_handle_future_.wait_for(std::chrono::milliseconds(0)) ==
	        std::future_status::ready) {
		const auto goal_handle = goal_handle_future_.get();
		if (goal_handle) {
			client_->async_cancel_goal(goal_handle);
		}
	} else {
		client_->async_cancel_all_goals();
	}
}
