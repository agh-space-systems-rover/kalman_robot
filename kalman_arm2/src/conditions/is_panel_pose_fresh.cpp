#include "conditions/is_panel_pose_fresh.hpp"

IsPanelPoseFresh::IsPanelPoseFresh(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SimpleConditionNode(
          name,
          std::bind(&IsPanelPoseFresh::tick, this),
          config
      ),
      parent_(parent) {
    panel_sub_ = parent_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "panel_pose",
        10,
        std::bind(
            &IsPanelPoseFresh::panel_pose_callback,
            this,
            std::placeholders::_1
        )
    );
}

BT::PortsList IsPanelPoseFresh::providedPorts() {
    return {BT::InputPort<double>("max_age_ms")};
}

void IsPanelPoseFresh::panel_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    last_panel_pose_ = *msg;
}

BT::NodeStatus IsPanelPoseFresh::tick() {
    if (!last_panel_pose_.has_value()) {
        return BT::NodeStatus::FAILURE;
    }

    const auto max_age_ms_input = getInput<double>("max_age_ms");
    const double max_age_ms =
        max_age_ms_input ? max_age_ms_input.value() : 300.0;
    const double age_ms = (
        parent_->now() - rclcpp::Time(last_panel_pose_->header.stamp)
    ).seconds() * 1000.0;

    return age_ms <= max_age_ms ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
}
