#include "conditions/is_panel_pose_available.hpp"

IsPanelPoseAvailable::IsPanelPoseAvailable(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SimpleConditionNode(
          name,
          std::bind(&IsPanelPoseAvailable::tick, this),
          config
      ),
      parent_(parent) {
    panel_sub_ = parent_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "panel_pose",
        10,
        std::bind(
            &IsPanelPoseAvailable::panel_pose_callback,
            this,
            std::placeholders::_1
        )
    );
}

BT::PortsList IsPanelPoseAvailable::providedPorts() {
    return {};
}

void IsPanelPoseAvailable::panel_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    last_panel_pose_ = *msg;
}

BT::NodeStatus IsPanelPoseAvailable::tick() {
    return last_panel_pose_.has_value() ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
}
