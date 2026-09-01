#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <optional>

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <kalman_interfaces/action/pose_ik_navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class PoseIKNavigateToPose : public BT::StatefulActionNode {
  public:
	using Action = kalman_interfaces::action::PoseIKNavigateToPose;
	using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

	PoseIKNavigateToPose(
	    const std::string &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void onHalted() override;

  private:
	rclcpp::Node *parent_;
	rclcpp_action::Client<Action>::SharedPtr client_;
	std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
	std::mutex mutex_;
	std::optional<rclcpp_action::ResultCode> last_result_;
	std::string last_message_;
	std::atomic<bool> halted_{false};
};
