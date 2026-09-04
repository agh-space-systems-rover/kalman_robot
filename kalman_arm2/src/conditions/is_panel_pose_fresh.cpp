#include "conditions/is_panel_pose_fresh.hpp"

IsPanelPoseFresh::IsPanelPoseFresh(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SimpleConditionNode(
          name, std::bind(&IsPanelPoseFresh::tick, this), config
      ),
      parent_(parent) {
	panel_sub_ = parent_->create_subscription<
	    geometry_msgs::msg::PoseWithCovarianceStamped>(
	    "panel_pose",
	    10,
	    std::bind(
	        &IsPanelPoseFresh::panel_pose_callback, this, std::placeholders::_1
	    )
	);
}

BT::PortsList IsPanelPoseFresh::providedPorts() {
	return {BT::InputPort<double>("max_age_ms")};
}

void IsPanelPoseFresh::panel_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
) {
	last_panel_pose_ = *msg;
}

BT::NodeStatus IsPanelPoseFresh::tick() {
	if (!last_panel_pose_.has_value()) {
		RCLCPP_WARN_STREAM(
		    parent_->get_logger(), name() << " failed: no panel pose available"
		);
		return BT::NodeStatus::FAILURE;
	}

	const auto   max_age_ms_input = getInput<double>("max_age_ms");
	const double max_age_ms =
	    max_age_ms_input ? max_age_ms_input.value() : 300.0;
	const double age_ms =
	    (parent_->now() - rclcpp::Time(last_panel_pose_->header.stamp))
	        .seconds() *
	    1000.0;

	if (age_ms > max_age_ms) {
		RCLCPP_WARN_STREAM(
		    parent_->get_logger(),
		    name() << " failed: panel pose too old (" << age_ms << " ms > "
		           << max_age_ms << " ms)"
		);
		return BT::NodeStatus::FAILURE;
	}
	return BT::NodeStatus::SUCCESS;
}
