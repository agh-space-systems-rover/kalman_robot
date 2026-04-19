#include "actions/compute_panel_target.hpp"

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ComputePanelTarget::ComputePanelTarget(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::SyncActionNode(name, config), parent_(parent) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList ComputePanelTarget::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>("panel_relative_pose"),
        BT::InputPort<double>("target_u"),
        BT::InputPort<double>("target_v"),
        BT::InputPort<double>("target_standoff"),
        BT::InputPort<std::string>("orientation_mode"),
        BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
    };
}

geometry_msgs::msg::Pose ComputePanelTarget::resolve_panel_relative_pose() const {
    const auto requested_pose =
        getInput<geometry_msgs::msg::Pose>("panel_relative_pose");
    const auto target_standoff_input = getInput<double>("target_standoff");
    if (requested_pose) {
        auto pose = requested_pose.value();
        if (target_standoff_input) {
            pose.position.z = target_standoff_input.value();
        }
        return pose;
    }

    const auto target_u_input = getInput<double>("target_u");
    const auto target_v_input = getInput<double>("target_v");

    geometry_msgs::msg::Pose panel_relative_pose;
    panel_relative_pose.position.x = target_u_input ? target_u_input.value() : 0.0;
    panel_relative_pose.position.y = target_v_input ? target_v_input.value() : 0.0;
    panel_relative_pose.position.z =
        target_standoff_input ? target_standoff_input.value() : 0.12;
    panel_relative_pose.orientation.w = 1.0;
    return panel_relative_pose;
}

BT::NodeStatus ComputePanelTarget::tick() {
    const auto orientation_mode_input = getInput<std::string>("orientation_mode");
    const std::string orientation_mode = orientation_mode_input
                                             ? orientation_mode_input.value()
                                             : "normal_to_panel";

    (void)orientation_mode;

    geometry_msgs::msg::TransformStamped base_to_panel;
    try {
        base_to_panel = tf_buffer_->lookupTransform(
            "base_link",
            "aruco_board",
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            parent_->get_logger(),
            "ComputePanelTarget failed to lookup panel TF: %s",
            ex.what()
        );
        return BT::NodeStatus::FAILURE;
    }

    const geometry_msgs::msg::Pose panel_relative_pose =
        resolve_panel_relative_pose();

    geometry_msgs::msg::Pose target_pose;
    tf2::doTransform(panel_relative_pose, target_pose, base_to_panel);

    (void)orientation_mode;

    setOutput("target_pose", target_pose);
    return BT::NodeStatus::SUCCESS;
}
