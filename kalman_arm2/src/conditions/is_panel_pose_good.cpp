#include "conditions/is_panel_pose_good.hpp"

#include <cmath>

IsPanelPoseGood::IsPanelPoseGood(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SimpleConditionNode(
          name,
          std::bind(&IsPanelPoseGood::tick, this),
          config
      ),
      parent_(parent) {
    panel_sub_ = parent_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "panel_pose",
        10,
        std::bind(
            &IsPanelPoseGood::panel_pose_callback,
            this,
            std::placeholders::_1
        )
    );
}

BT::PortsList IsPanelPoseGood::providedPorts() {
    return {
        BT::InputPort<double>("max_age_ms"),
        BT::InputPort<double>("max_position_error"),
        BT::InputPort<double>("max_normal_error_deg"),
    };
}

void IsPanelPoseGood::panel_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    last_panel_pose_ = *msg;
}

bool IsPanelPoseGood::pose_is_finite(
    const geometry_msgs::msg::PoseStamped &pose
) const {
    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(q.w);
}

BT::NodeStatus IsPanelPoseGood::tick() {
    if (!last_panel_pose_.has_value()) {
        return BT::NodeStatus::FAILURE;
    }

    const auto max_age_ms_input = getInput<double>("max_age_ms");
    const double max_age_ms =
        max_age_ms_input ? max_age_ms_input.value() : 300.0;
    const double age_ms = (
        parent_->now() - rclcpp::Time(last_panel_pose_->header.stamp)
    ).seconds() * 1000.0;

    if (age_ms > max_age_ms || !pose_is_finite(*last_panel_pose_)) {
        return BT::NodeStatus::FAILURE;
    }

    // Placeholder until panel_tracker publishes real quality metrics.
    (void)getInput<double>("max_position_error");
    (void)getInput<double>("max_normal_error_deg");

    return BT::NodeStatus::SUCCESS;
}
