#include "conditions/has_next_goal.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>

HasNextGoal::HasNextGoal(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SimpleConditionNode(
          name, std::bind(&HasNextGoal::tick, this), config
      ),
      parent_{parent} {}

BT::PortsList HasNextGoal::providedPorts() {
	return {BT::InputPort<std::shared_ptr<MissionHelper>>("mission_helper")};
}

BT::NodeStatus HasNextGoal::tick() {
	const auto mission_helper_opt =
	    getInput<std::shared_ptr<MissionHelper>>("mission_helper");

	if (!mission_helper_opt) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << "Mission helper does not exist"
		);
		return BT::NodeStatus::FAILURE;
	}
	if ((*mission_helper_opt).get() == nullptr) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << "Mission helper is nullptr"
		);
		return BT::NodeStatus::FAILURE;
	}
	const auto &mission_helper = mission_helper_opt.value();
  const auto unvisited_marker = mission_helper->get_unvisited_marker();
	if (unvisited_marker.has_value()) {
		return BT::NodeStatus::SUCCESS;
	}

	return BT::NodeStatus::FAILURE;
}
