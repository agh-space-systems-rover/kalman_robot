#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kalman_interfaces/action/move_to_panel_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace kalman_arm2 {

class PanelMissionTrigger : public rclcpp::Node {
  public:
    using MoveToPanelPose = kalman_interfaces::action::MoveToPanelPose;
    using GoalHandle =
        rclcpp_action::ClientGoalHandle<MoveToPanelPose>;

    explicit PanelMissionTrigger(const rclcpp::NodeOptions &options)
        : Node("panel_mission_trigger", options) {
        const std::string target_topic = declare_parameter<std::string>(
            "target_topic", "/arm/panel/target"
        );
        const std::string action_name = declare_parameter<std::string>(
            "action_name", "/arm/move_to_panel_pose"
        );
        target_z_ = declare_parameter<double>("target_z", 0.05);
        behavior_tree_ = declare_parameter<std::string>("behavior_tree", "demo");

        client_ = rclcpp_action::create_client<MoveToPanelPose>(
            this, action_name
        );
        if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_WARN(
                get_logger(),
                "Action server '%s' not available yet; goals will be retried on each target",
                action_name.c_str()
            );
        }

        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            target_topic,
            10,
            std::bind(
                &PanelMissionTrigger::on_target, this, std::placeholders::_1
            )
        );
    }

  private:
    void on_target(
        const geometry_msgs::msg::PoseStamped::SharedPtr message
    ) {
        const double x = message->pose.position.x;
        const double y = message->pose.position.y;
        if (!std::isfinite(x) || !std::isfinite(y)) {
            RCLCPP_WARN(get_logger(), "Ignoring non-finite panel target");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (active_goal_handle_) {
                RCLCPP_INFO(
                    get_logger(), "Cancelling in-flight goal for new target"
                );
                client_->async_cancel_goal(active_goal_handle_);
                active_goal_handle_.reset();
            }
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(
                get_logger(), "Action server not available; dropping target"
            );
            return;
        }

        MoveToPanelPose::Goal goal;
        goal.behavior_tree = behavior_tree_;
        goal.target_pose.position.x = x;
        goal.target_pose.position.y = y;
        goal.target_pose.position.z = target_z_;
        goal.target_pose.orientation.w = 1.0;

        auto send_opts =
            typename decltype(client_)::element_type::SendGoalOptions{};
        send_opts.goal_response_callback =
            [this](const GoalHandle::SharedPtr &handle) {
                if (!handle) {
                    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
                    return;
                }
                std::lock_guard<std::mutex> lock(goal_mutex_);
                active_goal_handle_ = handle;
            };
        send_opts.result_callback =
            [this](
                const rclcpp_action::Client<MoveToPanelPose>::WrappedResult
                    &result
            ) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Panel mission succeeded");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(get_logger(), "Panel mission canceled");
                        break;
                    default:
                        RCLCPP_WARN(
                            get_logger(), "Panel mission did not succeed"
                        );
                        break;
                }
                std::lock_guard<std::mutex> lock(goal_mutex_);
                active_goal_handle_.reset();
            };

        RCLCPP_INFO(
            get_logger(),
            "Panel target -> MoveToPanelPose goal: tree=%s, x=%.4f, y=%.4f, z=%.4f",
            behavior_tree_.c_str(),
            x,
            y,
            target_z_
        );
        client_->async_send_goal(goal, send_opts);
    }

    double target_z_{0.05};
    std::string behavior_tree_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        target_sub_;
    rclcpp_action::Client<MoveToPanelPose>::SharedPtr client_;

    std::mutex goal_mutex_;
    GoalHandle::SharedPtr active_goal_handle_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelMissionTrigger)
