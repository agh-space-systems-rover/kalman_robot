#include <cmath>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/action/arm_joint_move.hpp>
#include <kalman_interfaces/action/detail/arm_joint_move__struct.hpp>
#include <kalman_interfaces/msg/arm_joint_values.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

namespace kalman_arm2 {

class JointMove : public rclcpp::Node {
  public:
	// ROS interfaces
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
	rclcpp::Subscription<kalman_interfaces::msg::ArmJointValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmJointValues>::SharedPtr
	                             joint_vel_pub;
	rclcpp::TimerBase::SharedPtr compute_timer;

	using ArmJointMove = kalman_interfaces::action::ArmJointMove;
	using GoalHandleArmJointMove =
	    rclcpp_action::ServerGoalHandle<ArmJointMove>;
	rclcpp_action::Server<ArmJointMove>::SharedPtr action_server_;

	// TF2
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	// Parameters
	std::string base_link;
	std::string end_effector_link;
	float       max_joint_vel;
	double      update_rate;

	// Kinematics
	KDL::Chain                                  arm_chain;
	std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver;
	std::array<float, 6>                        current_joint_positions; // TODO: this is not KDL
	bool                                        joints_initialized;
	bool                                        kinematics_ready;

	// Latest twist message
	geometry_msgs::msg::TwistStamped::SharedPtr latest_twist;
	std::mutex                                  twist_mutex;

	JointMove(const rclcpp::NodeOptions &options)
	    : Node("joint_move", options), joints_initialized(false),
	      kinematics_ready(false) {
		this->declare_parameter<float>("max_joint_vel", 0.5);
		this->declare_parameter<double>("update_rate", 10.0);
		this->get_parameter("max_joint_vel", max_joint_vel);
		this->get_parameter("update_rate", update_rate);

		// Publishers & subscribers
		joint_vel_pub =
		    create_publisher<kalman_interfaces::msg::ArmJointValues>(
		        "target_vel", 10
		    );

		joint_pos_sub =
		    create_subscription<kalman_interfaces::msg::ArmJointValues>(
		        "current_pos",
		        10,
		        std::bind(
		            &JointMove::on_joint_positions, this, std::placeholders::_1
		        )
		    );

		// Create timer for periodic computation
		auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
		compute_timer     = create_wall_timer(
            timer_period, std::bind(&JointMove::test_joints, this)
        );

		// QoS profile for robot_description (latched topic)
		rclcpp::QoS robot_description_qos(1);
		robot_description_qos.transient_local();
		robot_description_qos.reliable();

		using namespace std::placeholders;
		action_server_ = rclcpp_action::create_server<ArmJointMove>(
		    this,
		    "move_arm",
		    std::bind(&JointMove::handle_goal, this, _1, _2),
		    std::bind(&JointMove::handle_cancel, this, _1),
		    std::bind(&JointMove::handle_accepted, this, _1)
		);
	}

	rclcpp_action::GoalResponse handle_goal(
	    const rclcpp_action::GoalUUID            &uuid,
	    std::shared_ptr<const ArmJointMove::Goal> goal
	) {
		(void)uuid;
		RCLCPP_INFO(
		    this->get_logger(),
		    "Received goal request joints"
		);
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	// Handle cancellations
	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<GoalHandleArmJointMove> goal_handle) {
		(void)goal_handle;
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	// Start processing the goal
	void
	handle_accepted(const std::shared_ptr<GoalHandleArmJointMove> goal_handle) {
		// Offload to a new thread
		std::thread{[=]() {
			this->execute(goal_handle);
		}}.detach();
	}
	void execute(const std::shared_ptr<GoalHandleArmJointMove> goal_handle) {
		RCLCPP_INFO(this->get_logger(), "Executing goal");
		const std::shared_ptr<const ArmJointMove::Goal> goal = goal_handle->get_goal();

		auto       feedback = std::make_shared<ArmJointMove::Feedback>();
		auto      &current  = feedback->current_joint_values;

		rclcpp::Rate loop_rate(10);
		bool         success = true;

		const kalman_interfaces::msg::ArmJointValues& target_pose = goal->target_joint_values;

		while (true) {
			while (!joints_initialized){
				RCLCPP_INFO(this->get_logger(), "Waiting for joints state msg");
			}

			auto vel_msg         = kalman_interfaces::msg::ArmJointValues();
			vel_msg.header.stamp = now();

			constexpr double EPS = 0.01;
			bool all_below_eps = true;

			for (size_t i = 0; i < vel_msg.joints.size(); i++){
				goal->target_joint_values;
				const auto current = current_joint_positions.at(i);
				const auto& target = target_pose.joints.at(i);
				const float delta = target - current;
				// const float delta = current - target;

				double v = std::min(double{max_joint_vel}, std::abs(delta * 2.0));
				v = std::copysign(v, delta);
				// double v = delta;

				RCLCPP_INFO(this->get_logger(), "Joint %zu, current: %f, target %f, v: %f", i, current, target, v);

				vel_msg.joints.at(i) = v;
				if (target == -1.0) {
					vel_msg.joints.at(i) = 0.0;
				}
				all_below_eps &= std::abs(v) < EPS;
			}
			if (all_below_eps){
				break;
			}

			vel_msg.jaw = 0.0;

			joint_vel_pub->publish(vel_msg);

			loop_rate.sleep();
		}

		// Finally, set the result
		auto result     = std::make_shared<ArmJointMove::Result>();
		result->success = success;
		result->message = success ? "Move completed" : "Move failed";
		goal_handle->succeed(result);

		RCLCPP_INFO(this->get_logger(), "Goal succeeded");
	}

	void on_joint_positions(
	    const kalman_interfaces::msg::ArmJointValues::SharedPtr msg
	) {
		current_joint_positions = msg->joints;

		joints_initialized = true;
	}

	void test_joints() {

	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::JointMove)
