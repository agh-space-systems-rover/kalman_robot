#include "actions/visual_refine_to_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

tf2::Transform poseToTf(const geometry_msgs::msg::Pose &pose) {
    tf2::Transform transform;
    tf2::fromMsg(pose, transform);
    return transform;
}

tf2::Transform transformToTf(const geometry_msgs::msg::Transform &transform_msg) {
    tf2::Transform transform;
    tf2::fromMsg(transform_msg, transform);
    return transform;
}

double rotationAngle(tf2::Quaternion rotation) {
    rotation.normalize();
    if (rotation.w() < 0.0) {
        rotation = tf2::Quaternion(
            -rotation.x(), -rotation.y(), -rotation.z(), -rotation.w()
        );
    }
    return 2.0 * std::acos(
        std::clamp(static_cast<double>(rotation.w()), -1.0, 1.0)
    );
}

tf2::Transform boundedCorrection(
    const tf2::Transform &correction,
    bool refine_orientation,
    double max_translation_step,
    double max_rotation_step_rad
) {
    tf2::Transform bounded = correction;

    tf2::Vector3 translation = correction.getOrigin();
    const double translation_length = translation.length();
    if (translation_length > max_translation_step) {
        translation *= max_translation_step / translation_length;
    }
    bounded.setOrigin(translation);

    if (!refine_orientation) {
        bounded.setRotation(tf2::Quaternion::getIdentity());
        return bounded;
    }

    tf2::Quaternion rotation = correction.getRotation();
    rotation.normalize();
    if (rotation.w() < 0.0) {
        rotation = tf2::Quaternion(
            -rotation.x(), -rotation.y(), -rotation.z(), -rotation.w()
        );
    }
    const double angle = rotationAngle(rotation);
    if (angle > max_rotation_step_rad) {
        rotation = tf2::Quaternion::getIdentity().slerp(
            rotation, max_rotation_step_rad / angle
        );
        rotation.normalize();
    }
    bounded.setRotation(rotation);
    return bounded;
}

} // namespace

VisualRefineToPanel::VisualRefineToPanel(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node *parent
)
    : BT::StatefulActionNode(name, config), parent_(parent) {
    visual_sub_ = parent_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "visual_ee_pose",
        10,
        std::bind(
            &VisualRefineToPanel::visual_pose_callback,
            this,
            std::placeholders::_1
        )
    );
    action_client_ = rclcpp_action::create_client<Action>(
        parent_->get_node_base_interface(),
        parent_->get_node_graph_interface(),
        parent_->get_node_logging_interface(),
        parent_->get_node_waitables_interface(),
        "pose_ik_navigate_to_pose"
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    parent_->get_parameter("visual_refinement_dof", refinement_dof_);
    parent_->get_parameter("visual_refinement_max_corrections", max_corrections_);
    parent_->get_parameter(
        "visual_refinement_max_measurement_age_s", max_measurement_age_s_
    );
    parent_->get_parameter("visual_refinement_settle_time_s", settle_time_s_);
    parent_->get_parameter(
        "visual_refinement_max_translation_step", max_translation_step_
    );

    double max_rotation_step_deg = 5.0;
    double visual_orientation_tolerance_deg = 3.0;
    parent_->get_parameter(
        "visual_refinement_max_rotation_step_deg", max_rotation_step_deg
    );
    parent_->get_parameter(
        "visual_refinement_position_tolerance", visual_position_tolerance_
    );
    parent_->get_parameter(
        "visual_refinement_orientation_tolerance_deg",
        visual_orientation_tolerance_deg
    );
    parent_->get_parameter("visual_refinement_base_frame", base_frame_);
    parent_->get_parameter("visual_refinement_ee_frame", ee_frame_);
    parent_->get_parameter("visual_refinement_panel_frame", panel_frame_);

    max_rotation_step_rad_ = max_rotation_step_deg * M_PI / 180.0;
    visual_orientation_tolerance_rad_ =
        visual_orientation_tolerance_deg * M_PI / 180.0;
}

BT::PortsList VisualRefineToPanel::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>(
            "pose", "Literal desired arm_link_gripper pose in the panel frame"
        ),
        BT::InputPort<double>("timeout_ms", "Timeout in milliseconds"),
    };
}

void VisualRefineToPanel::visual_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(visual_pose_mutex_);
    latest_visual_pose_ = *msg;
}

BT::NodeStatus VisualRefineToPanel::onStart() {
    const auto desired_pose = getInput<geometry_msgs::msg::Pose>("pose");
    if (!desired_pose) {
        RCLCPP_ERROR(parent_->get_logger(), "%s has no target pose", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    desired_panel_pose_ = desired_pose.value();

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(
            parent_->get_logger(),
            "%s: pose IK navigation action server is unavailable",
            name().c_str()
        );
        return BT::NodeStatus::FAILURE;
    }

    ++action_generation_;
    correction_goal_active_ = false;
    {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        action_result_.reset();
        action_result_message_.clear();
    }

    const auto timeout = getInput<double>("timeout_ms");
    const double timeout_ms = timeout ? timeout.value() : 15000.0;
    deadline_ =
        parent_->now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
    last_used_measurement_stamp_ = rclcpp::Time(
        0, 0, parent_->get_clock()->get_clock_type()
    );
    correction_count_ = 0;
    state_ = State::WAITING_FOR_MEASUREMENT;

    if (refinement_dof_ != 3 && refinement_dof_ != 6) {
        RCLCPP_ERROR(
            parent_->get_logger(),
            "visual_refinement_dof must be 3 or 6, got %d",
            refinement_dof_
        );
        return BT::NodeStatus::FAILURE;
    }
    if (!std::isfinite(max_measurement_age_s_) || max_measurement_age_s_ <= 0.0 ||
        !std::isfinite(settle_time_s_) || settle_time_s_ < 0.0 ||
        !std::isfinite(max_translation_step_) || max_translation_step_ <= 0.0 ||
        !std::isfinite(max_rotation_step_rad_) || max_rotation_step_rad_ <= 0.0 ||
        max_corrections_ <= 0) {
        RCLCPP_ERROR(parent_->get_logger(), "Invalid visual refinement parameters");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

std::optional<geometry_msgs::msg::PoseStamped>
VisualRefineToPanel::visual_pose_snapshot() const {
    std::lock_guard<std::mutex> lock(visual_pose_mutex_);
    return latest_visual_pose_;
}

const char *VisualRefineToPanel::state_name() const {
    switch (state_) {
    case State::WAITING_FOR_MEASUREMENT:
        return "waiting_for_measurement";
    case State::MOVING:
        return "moving_to_correction";
    case State::SETTLING:
        return "settling";
    }
    return "unknown";
}

std::optional<tf2::Transform> VisualRefineToPanel::current_base_to_ee() const {
    try {
        const auto transform = tf_buffer_->lookupTransform(
            base_frame_,
            ee_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)
        );
        return transformToTf(transform.transform);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            1000,
            "Visual refinement TF lookup failed: %s",
            ex.what()
        );
        return std::nullopt;
    }
}

BT::NodeStatus VisualRefineToPanel::begin_correction() {
    const auto visual_pose = visual_pose_snapshot();
    if (!visual_pose) {
        RCLCPP_ERROR_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            2000,
            "Visual refinement has received no EE pose on 'visual_ee_pose'. "
            "Marker 30 or 31 and at least one panel marker must be detected "
            "in the same camera frame."
        );
        return BT::NodeStatus::RUNNING;
    }

    const rclcpp::Time measurement_stamp(visual_pose->header.stamp);
    const double measurement_age_s =
        (parent_->now() - measurement_stamp).seconds();
    if (measurement_age_s < 0.0) {
        RCLCPP_ERROR_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            2000,
            "Visual EE pose timestamp is %.3f s in the future; check ROS clocks",
            -measurement_age_s
        );
        return BT::NodeStatus::RUNNING;
    }
    if (measurement_age_s > max_measurement_age_s_) {
        RCLCPP_ERROR_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            2000,
            "Visual EE pose is stale (age %.3f s, limit %.3f s). Marker 31 or all panel markers are likely no longer detected.",
            measurement_age_s,
            max_measurement_age_s_
        );
        return BT::NodeStatus::RUNNING;
    }
    if (measurement_stamp <= last_used_measurement_stamp_) {
        RCLCPP_WARN_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            2000,
            "Waiting for a new visual EE pose captured after the previous correction; latest stamp is %.3f s before the required cutoff",
            (last_used_measurement_stamp_ - measurement_stamp).seconds()
        );
        return BT::NodeStatus::RUNNING;
    }

    if (visual_pose->header.frame_id != panel_frame_) {
        RCLCPP_ERROR_THROTTLE(
            parent_->get_logger(),
            *parent_->get_clock(),
            1000,
            "Visual EE pose uses frame '%s', expected '%s'",
            visual_pose->header.frame_id.c_str(),
            panel_frame_.c_str()
        );
        return BT::NodeStatus::RUNNING;
    }

    const tf2::Transform measured_panel_to_ee =
        poseToTf(visual_pose->pose);
    const tf2::Transform desired_panel_to_ee = poseToTf(desired_panel_pose_);
    const tf2::Transform correction =
        measured_panel_to_ee.inverse() * desired_panel_to_ee;

    const double position_error = correction.getOrigin().length();
    const double orientation_error = rotationAngle(correction.getRotation());
    const bool refine_orientation = refinement_dof_ == 6;
    if (position_error <= visual_position_tolerance_ &&
        (!refine_orientation ||
         orientation_error <= visual_orientation_tolerance_rad_)) {
        RCLCPP_INFO(
            parent_->get_logger(),
            "Visual refinement converged after %d corrections: position=%.4f m, rotation=%.2f deg",
            correction_count_,
            position_error,
            orientation_error * 180.0 / M_PI
        );
        return BT::NodeStatus::SUCCESS;
    }

    if (correction_count_ >= max_corrections_) {
        RCLCPP_ERROR(
            parent_->get_logger(),
            "Visual refinement failed after reaching limit of %d corrections: remaining position error=%.4f m, rotation error=%.2f deg",
            max_corrections_,
            position_error,
            orientation_error * 180.0 / M_PI
        );
        return BT::NodeStatus::FAILURE;
    }

    const auto base_to_ee = current_base_to_ee();
    if (!base_to_ee) {
        return BT::NodeStatus::RUNNING;
    }

    commanded_base_to_ee_ = *base_to_ee * boundedCorrection(
        correction,
        refine_orientation,
        max_translation_step_,
        max_rotation_step_rad_
    );
    last_used_measurement_stamp_ = rclcpp::Time(visual_pose->header.stamp);
    ++correction_count_;
    state_ = State::MOVING;
    if (!send_commanded_pose()) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

bool VisualRefineToPanel::send_commanded_pose() {
    const double remaining_seconds = (deadline_ - parent_->now()).seconds();
    if (remaining_seconds <= 0.0) {
        return false;
    }

    Action::Goal goal;
    goal.pose.position.x = commanded_base_to_ee_.getOrigin().x();
    goal.pose.position.y = commanded_base_to_ee_.getOrigin().y();
    goal.pose.position.z = commanded_base_to_ee_.getOrigin().z();
    goal.pose.orientation = tf2::toMsg(commanded_base_to_ee_.getRotation());
    goal.base_frame = base_frame_;
    goal.end_effector_link = ee_frame_;
    goal.timeout_seconds = remaining_seconds;

    const uint64_t generation = ++action_generation_;
    {
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        action_result_.reset();
        action_result_message_.clear();
    }

    auto options = rclcpp_action::Client<Action>::SendGoalOptions{};
    options.result_callback = [this, generation](
        const GoalHandle::WrappedResult &result
    ) {
        if (generation != action_generation_.load()) {
            return;
        }
        correction_goal_active_ = false;
        std::lock_guard<std::mutex> lock(action_result_mutex_);
        action_result_ = result.code;
        if (result.result) {
            action_result_message_ = result.result->message;
        }
    };
    correction_goal_active_ = true;
    try {
        goal_handle_future_ = action_client_->async_send_goal(goal, options);
    } catch (const std::exception &error) {
        correction_goal_active_ = false;
        RCLCPP_ERROR(
            parent_->get_logger(),
            "%s: failed to send pose IK correction goal: %s",
            name().c_str(),
            error.what()
        );
        return false;
    }
    return true;
}

void VisualRefineToPanel::cancel_active_goal() {
    if (!correction_goal_active_.exchange(false)) {
        return;
    }
    ++action_generation_;
    try {
        if (goal_handle_future_.valid() &&
            goal_handle_future_.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready) {
            const auto goal_handle = goal_handle_future_.get();
            if (goal_handle) {
                action_client_->async_cancel_goal(goal_handle);
            }
        } else {
            action_client_->async_cancel_all_goals();
        }
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
        // Goal reached a terminal state between our active check and cancel.
    }
}

BT::NodeStatus VisualRefineToPanel::onRunning() {
    if (parent_->now() >= deadline_) {
        const auto visual_pose = visual_pose_snapshot();
        if (!visual_pose) {
            RCLCPP_ERROR(
                parent_->get_logger(),
                "%s timed out in state '%s' after %d corrections: no visual EE pose was ever received. Verify marker 31 and at least one panel marker are visible.",
                name().c_str(),
                state_name(),
                correction_count_
            );
        } else {
            const double age_s =
                (parent_->now() - rclcpp::Time(visual_pose->header.stamp)).seconds();
            RCLCPP_ERROR(
                parent_->get_logger(),
                "%s timed out in state '%s' after %d corrections; latest visual EE pose age is %.3f s (limit %.3f s)",
                name().c_str(),
                state_name(),
                correction_count_,
                age_s,
                max_measurement_age_s_
            );
        }
        cancel_active_goal();
        return BT::NodeStatus::FAILURE;
    }

    switch (state_) {
    case State::WAITING_FOR_MEASUREMENT:
        return begin_correction();
    case State::MOVING: {
        if (goal_handle_future_.valid() &&
            goal_handle_future_.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready &&
            !goal_handle_future_.get()) {
            correction_goal_active_ = false;
            RCLCPP_ERROR(
                parent_->get_logger(),
                "%s: pose IK correction goal was rejected",
                name().c_str()
            );
            return BT::NodeStatus::FAILURE;
        }

        std::optional<rclcpp_action::ResultCode> result;
        std::string result_message;
        {
            std::lock_guard<std::mutex> lock(action_result_mutex_);
            result = action_result_;
            result_message = action_result_message_;
        }
        if (!result) {
            return BT::NodeStatus::RUNNING;
        }
        if (*result != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(
                parent_->get_logger(),
                "%s: pose IK correction failed: %s",
                name().c_str(),
                result_message.c_str()
            );
            return BT::NodeStatus::FAILURE;
        }

        settle_deadline_ = parent_->now() +
                           rclcpp::Duration::from_seconds(settle_time_s_);
        state_ = State::SETTLING;
        return BT::NodeStatus::RUNNING;
    }
    case State::SETTLING:
        if (parent_->now() >= settle_deadline_) {
            last_used_measurement_stamp_ = parent_->now();
            state_ = State::WAITING_FOR_MEASUREMENT;
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;
}

void VisualRefineToPanel::onHalted() {
    cancel_active_goal();
}
