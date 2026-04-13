#include "actions/acquire_panel_pose.hpp"

AcquirePanelPose::AcquirePanelPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
    panel_sub_ = parent_->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "panel_pose",
        10,
        std::bind(
            &AcquirePanelPose::panel_pose_callback,
            this,
            std::placeholders::_1
        )
    );
}

BT::PortsList AcquirePanelPose::providedPorts() {
    return {
        BT::InputPort<double>("timeout_ms"),
        BT::InputPort<double>("max_age_ms"),
    };
}

void AcquirePanelPose::panel_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
) {
    last_panel_pose_ = *msg;
}

BT::NodeStatus AcquirePanelPose::onStart() {
    const auto timeout_input = getInput<double>("timeout_ms");
    const auto freshness_input = getInput<double>("max_age_ms");
    const double timeout_ms = timeout_input ? timeout_input.value() : 3000.0;
    freshness_ms_ = freshness_input ? freshness_input.value() : 300.0;
    deadline_ = parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AcquirePanelPose::onRunning() {
    if (last_panel_pose_.has_value()) {
        const double age_ms = (
            parent_->now() - rclcpp::Time(last_panel_pose_->header.stamp)
        ).seconds() * 1000.0;
        if (age_ms <= freshness_ms_) {
            return BT::NodeStatus::SUCCESS;
        }
    }

    if (parent_->now() >= deadline_) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void AcquirePanelPose::onHalted() {}
