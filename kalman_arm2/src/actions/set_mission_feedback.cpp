#include "actions/set_mission_feedback.hpp"

#include "mission_feedback.hpp"

SetMissionFeedback::SetMissionFeedback(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SyncActionNode(name, config), parent_(parent) {}

BT::PortsList SetMissionFeedback::providedPorts() {
	return {BT::InputPort<std::string>("text")};
}

BT::NodeStatus SetMissionFeedback::tick() {
	const auto text_input = getInput<std::string>("text");
	if (!text_input.has_value()) {
		throw BT::RuntimeError(name(), " missing required input [text]");
	}

	auto *feedback_handle = dynamic_cast<MissionFeedbackHandle *>(parent_);
	if (!feedback_handle) {
		RCLCPP_WARN_STREAM(
		    parent_->get_logger(),
		    name() << " failed: parent does not implement MissionFeedbackHandle"
		);
		return BT::NodeStatus::FAILURE;
	}

	feedback_handle->publish_progress(text_input.value());
	return BT::NodeStatus::SUCCESS;
}
