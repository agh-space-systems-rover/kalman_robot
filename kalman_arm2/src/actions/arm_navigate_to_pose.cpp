#include "actions/arm_navigate_to_pose.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <bitset>
#include <chrono>
#include <cstdlib>
#include <kalman_interfaces/action/detail/arm_goto_joint_pose__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_client.hpp>
#include <string>

ArmNavigateToPose::ArmNavigateToPose(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config),
    action_node_{std::make_shared<rclcpp::Node>("action_node")},
    parent_{parent} {
	client_ = rclcpp_action::create_client<
	    kalman_interfaces::action::ArmGotoJointPose>(parent_, "goto_pose");
}

BT::PortsList ArmNavigateToPose::providedPorts() {
	return {
	    BT::InputPort<double>("j0", "Joint 0 position"),
	    BT::InputPort<double>("j1", "Joint 1 position"),
	    BT::InputPort<double>("j2", "Joint 2 position"),
	    BT::InputPort<double>("j3", "Joint 3 position"),
	    BT::InputPort<double>("j4", "Joint 4 position"),
	    BT::InputPort<double>("j5", "Joint 5 position"),
	    BT::InputPort<double>("jaw", "Gripper jaw position")
	};
}

BT::NodeStatus ArmNavigateToPose::onStart() {
	RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] tick()");

	if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
		RCLCPP_ERROR(parent_->get_logger(), "Nav action server not available");
		return BT::NodeStatus::FAILURE;
	}

	std::array<float, 6> target_joints{};
	float                jaw{0.0};
	std::bitset<7>       ignore_mask{}; // Bit of index 6 is jaw

	for (size_t i = 0; i < 6; i++) {
		const std::string key = "j" + std::to_string(i);
		const auto        opt = getInput<double>(key);
		ignore_mask[i]        = !opt.has_value();
		if (opt.has_value()) {
			target_joints.at(i) = opt.value();
		}
	}
	const auto opt = getInput<double>("jaw");
	{
		ignore_mask[6] = !opt.has_value();
		if (opt.has_value()) {
			jaw = opt.value();
		}
	}
	target_joints_ = target_joints;

	kalman_interfaces::action::ArmGotoJointPose::Goal goal;
	goal.target_pos.joints = target_joints;
	// goal.target_pos.jaw = jaw;
	goal.target_pos.jaw = 0.0;
	goal.ignore_mask       = ignore_mask.to_ulong();

	auto send_opts =
	    typename decltype(client_)::element_type::SendGoalOptions{};
	send_opts.result_callback =
	    [this](const decltype(client_)::element_type::WrappedResult &r) {
		    std::lock_guard<std::mutex> lk(m_);
		    last_result_ = r.code; // store for onRunning
	    };
	last_result_ = {};

	goal_handle_future_ = client_->async_send_goal(goal, send_opts);

	RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onStart() returning");

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmNavigateToPose::onRunning() {
  RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onRunning()");
	// If cancellation was requested via halt(), report SUCCESS so the Parallel
	// can finish.
	if (cancelled_) {
    RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onRunning() CANCELLED");
		return BT::NodeStatus::SUCCESS;
	}
	// If goal finished, convert result to SUCCESS/FAILURE
	std::lock_guard<std::mutex> lk(m_);
	if (last_result_) {
		if (*last_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onRunning() returning SUCCESS");
			return BT::NodeStatus::SUCCESS;
		} else {
      RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onRunning() returning FAILURE");
			return BT::NodeStatus::FAILURE;
		}
	}
	std::string joints_str = "";
	for (const auto j : target_joints_){
		joints_str += std::to_string(j) + " ";
	}
	RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onRunning() returning RUNNING to %s", joints_str.c_str());
	return BT::NodeStatus::RUNNING;
}

void ArmNavigateToPose::onHalted() {
  RCLCPP_INFO(parent_->get_logger(), "[ArmNavigateToPose] onHalted()");
	client_->async_cancel_all_goals();
	// Cancel active goal when the subtree is halted (e.g., monitor trips)
	cancelled_ = true;

	// If we already have a goal handle, cancel it
	try {
		if (goal_handle_future_.valid()) {
			// Get goal handle if already available
			if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) ==
			    std::future_status::ready) {
				auto gh = goal_handle_future_.get();
				if (gh) {
					(void)client_->async_cancel_goal(gh);
				}
			}
		}
	} catch (...) {
		// ignore edge cases (future already consumed, etc.)
	}
}
