#include <cmath>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/action/arm_goto_joint_pose.hpp>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/msg/string.hpp>

namespace kalman_arm2 {

template <typename T> auto &ith_joint_val(T &msg, size_t i) {
	if (i < 6) {
		return msg.joints.at(i);
	} else {
		return msg.jaw;
	}
}

class GotoJointPose : public rclcpp::Node {
  public:
	// ROS interfaces
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr
	                             joint_vel_pub;
	rclcpp::TimerBase::SharedPtr timer;

	using ArmGotoJointPose = kalman_interfaces::action::ArmGotoJointPose;
	using GoalHandleArmGotoJointPose =
	    rclcpp_action::ServerGoalHandle<ArmGotoJointPose>;
	rclcpp_action::Server<ArmGotoJointPose>::SharedPtr action_server_;

	// TF2
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	// Parameters
	float  max_joint_vel;
	double update_rate;
	float  max_error;

	// State
	kalman_interfaces::msg::ArmValues      current_pos;
	std::shared_ptr<GoalHandleArmGotoJointPose> current_gh;

	GotoJointPose(const rclcpp::NodeOptions &options)
	    : Node("goto_joint_pose", options) {
		// Parameters
		this->declare_parameter<float>("max_joint_vel", 0.5);
		this->declare_parameter<double>("update_rate", 10.0);
		this->declare_parameter<float>("max_error", 0.1);
		this->get_parameter("max_joint_vel", max_joint_vel);
		this->get_parameter("update_rate", update_rate);
		this->get_parameter("max_error", max_error);

		// Publishers & subscribers
		joint_vel_pub =
		    create_publisher<kalman_interfaces::msg::ArmValues>(
		        "target_vel", 10
		    );
		joint_pos_sub =
		    create_subscription<kalman_interfaces::msg::ArmValues>(
		        "current_pos",
		        10,
		        std::bind(
		            &GotoJointPose::on_joint_positions,
		            this,
		            std::placeholders::_1
		        )
		    );

		// Create timer for periodic computation
		auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
		timer             = create_wall_timer(
            timer_period, std::bind(&GotoJointPose::timer_cb, this)
        );

		using namespace std::placeholders;
		action_server_ = rclcpp_action::create_server<ArmGotoJointPose>(
		    this,
		    "goto_pose",
		    std::bind(&GotoJointPose::handle_goal, this, _1, _2),
		    std::bind(&GotoJointPose::handle_cancel, this, _1),
		    std::bind(&GotoJointPose::handle_accepted, this, _1)
		);
	}

	rclcpp_action::GoalResponse handle_goal(
	    const rclcpp_action::GoalUUID                &uuid,
	    std::shared_ptr<const ArmGotoJointPose::Goal> goal
	) {
		RCLCPP_INFO(get_logger(), "Received goal request joints");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<GoalHandleArmGotoJointPose> gh) {
		RCLCPP_INFO(get_logger(), "Received request to cancel goal");
		if (current_gh == gh) {
			current_gh.reset();
			RCLCPP_INFO(get_logger(), "Goal cancelled");
		} else {
			RCLCPP_WARN(
			    get_logger(),
			    "Cancel request for a goal that is not currently active"
			);
		}
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	void handle_accepted(const std::shared_ptr<GoalHandleArmGotoJointPose> gh) {
		RCLCPP_INFO(this->get_logger(), "Accepted goal");
		current_gh = gh;
	}

	void on_joint_positions(
	    const kalman_interfaces::msg::ArmValues::SharedPtr msg
	) {
		current_pos = *msg;
	}

	void timer_cb() {
		if (!current_gh) {
			return;
		}

		auto vel_msg         = kalman_interfaces::msg::ArmValues();
		vel_msg.header.stamp = now();
		vel_msg.joints.fill(0.0);
		vel_msg.jaw = 0.0;

		uint8_t mask     = current_gh->get_goal()->ignore_mask;
		bool    all_done = true;
		for (size_t i = 0; i < 7; i++) {
			// mask check
			if ((mask & (1 << i))) {
				continue;
			}

			auto &target_pos = current_gh->get_goal()->target_pos;

			// error
			const float current = ith_joint_val(current_pos, i);
			const float target  = ith_joint_val(target_pos, i);
			const float error   = target - current;
			if (std::abs(error) > max_error) {
				all_done = false;

				// velocity
				double vel = std::min(max_joint_vel, std::abs(error * 1.0f));
				vel        = std::copysign(vel, error);
				ith_joint_val(vel_msg, i) = vel;
			}
		}

		joint_vel_pub->publish(vel_msg);

		// Decide if done
		if (all_done) {
			current_gh->succeed(std::make_shared<ArmGotoJointPose::Result>());
			RCLCPP_INFO(get_logger(), "Goal succeeded");
			current_gh.reset();
		} else {
			auto fb         = std::make_shared<ArmGotoJointPose::Feedback>();
			fb->current_pos = current_pos;
			current_gh->publish_feedback(fb);
		}
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::GotoJointPose)
