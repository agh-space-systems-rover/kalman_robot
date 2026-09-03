#include "actions/say_something.hpp"

#include "mission_feedback.hpp"

SaySomething::SaySomething(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SyncActionNode(name, config), parent_{parent} {}

BT::PortsList SaySomething::providedPorts() {
	return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus SaySomething::tick() {
	const auto message = getInput<std::string>("message").value_or("nothing");
	RCLCPP_ERROR_STREAM(parent_->get_logger(), name() << ": " << message);
	if (auto *feedback_handle = dynamic_cast<MissionFeedbackHandle *>(parent_)) {
		feedback_handle->publish_progress(message);
	}
	// std::this_thread::sleep_for(std::chrono::milliseconds(50));
	return BT::NodeStatus::SUCCESS;
}