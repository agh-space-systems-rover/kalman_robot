#include <chrono>
#include <cmath>
#include <control_msgs/msg/joint_jog.hpp>
#include <kalman_interfaces/msg/arm_joint_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace kalman_arm2 {

class Arm1MoveitCompatNode : public rclcpp::Node {
  public:
	// control
	rclcpp::Subscription<kalman_interfaces::msg::ArmJointValues>::SharedPtr
	    target_joint_vel_sub; // arm/joints/target_vel
	rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr
	    joint_jog_pub; // servo_node/delta_joint_cmds
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
	                             gripper_cmd; // gripper/command_incremental
	rclcpp::TimerBase::SharedPtr control_timer;

	// feedback
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
	    joint_state_sub; // arm_controllers/joint_states
	rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr
	    gripper_pos_sub; // gripper/position
	rclcpp::Publisher<kalman_interfaces::msg::ArmJointValues>::SharedPtr
	    joint_pos_pub; // arm/joints/current_pos

	uint16_t gripper_open_pos    = 2000;
	uint16_t gripper_closed_pos  = 3000;
	int      gripper_cmd_per_deg = 5;

	float                                  control_timeout = 0.1;  // seconds
	float                                  control_rate    = 10.0; // Hz
	kalman_interfaces::msg::ArmJointValues last_target_joint_vel;
	rclcpp::Time                           last_target_joint_vel_time;
	rclcpp::Time                           last_control_pub_time;

	sensor_msgs::msg::JointState last_joint_state;
	std_msgs::msg::UInt16        last_gripper_pos;

	Arm1MoveitCompatNode(const rclcpp::NodeOptions &options)
	    : Node("arm1_moveit_compat", options), last_target_joint_vel_time(this->now()), last_control_pub_time(this->now()) {

		// Initialize subscribers and publishers

		// control
		target_joint_vel_sub =
		    this->create_subscription<kalman_interfaces::msg::ArmJointValues>(
		        "arm/joints/target_vel",
		        10,
		        std::bind(
		            &Arm1MoveitCompatNode::target_joint_vel_cb,
		            this,
		            std::placeholders::_1
		        )
		    );
		joint_jog_pub = this->create_publisher<control_msgs::msg::JointJog>(
		    "servo_node/delta_joint_cmds", 10
		);
		gripper_cmd = this->create_publisher<std_msgs::msg::Int8>(
		    "gripper/command_incremental", 10
		);
		control_timer = this->create_wall_timer(
		    std::chrono::duration<double>(1.0f / control_rate),
		    std::bind(&Arm1MoveitCompatNode::control_timer_cb, this)
		);

		// feedback
		joint_state_sub =
		    this->create_subscription<sensor_msgs::msg::JointState>(
		        "arm_controllers/joint_states",
		        10,
		        std::bind(
		            &Arm1MoveitCompatNode::joint_state_cb,
		            this,
		            std::placeholders::_1
		        )
		    );
		gripper_pos_sub = this->create_subscription<std_msgs::msg::UInt16>(
		    "gripper/position",
		    10,
		    std::bind(
		        &Arm1MoveitCompatNode::gripper_pos_cb,
		        this,
		        std::placeholders::_1
		    )
		);
		joint_pos_pub =
		    this->create_publisher<kalman_interfaces::msg::ArmJointValues>(
		        "arm/joints/current_pos", 10
		    );
	}

	void target_joint_vel_cb(
	    const kalman_interfaces::msg::ArmJointValues::SharedPtr msg
	) {
		last_target_joint_vel      = *msg;
		last_target_joint_vel_time = this->now();
	}

	void control_timer_cb() {
        // timeout
		if ((this->now() - last_target_joint_vel_time).seconds() >
		    control_timeout) {
			return;
		}

        // delta time
        rclcpp::Time now = this->now();
        double delta_time = (now - last_control_pub_time).seconds();
        last_control_pub_time = now;

		// 6-DoF
		control_msgs::msg::JointJog jog_msg;
		jog_msg.header.stamp = this->now();
		jog_msg.joint_names  = {
            "arm_joint_1",
            "arm_joint_2",
            "arm_joint_3",
            "arm_joint_4",
            "arm_joint_5",
            "arm_joint_6"
        };
		jog_msg.velocities = {
		    last_target_joint_vel.joints[0],
		    last_target_joint_vel.joints[1],
		    last_target_joint_vel.joints[2],
		    last_target_joint_vel.joints[3],
		    last_target_joint_vel.joints[4],
		    last_target_joint_vel.joints[5]
		};
		joint_jog_pub->publish(jog_msg);
		// Gripper control
		std_msgs::msg::Int8 gripper_msg;
		gripper_msg.data =
		    gripper_cmd_per_deg * (last_target_joint_vel.jaw * 180 / M_PI) * delta_time;
		gripper_cmd->publish(gripper_msg);
	}

	void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
		last_joint_state = *msg;
		pub_feedback();
	}

	void gripper_pos_cb(const std_msgs::msg::UInt16::SharedPtr msg) {
		last_gripper_pos = *msg;
		pub_feedback();
	}

	void pub_feedback() {
		kalman_interfaces::msg::ArmJointValues joint_msg;
		joint_msg.header.stamp    = this->now();
		joint_msg.header.frame_id = "";
		joint_msg.joints[0]       = last_joint_state.position[0];
		joint_msg.joints[1]       = last_joint_state.position[1];
		joint_msg.joints[2]       = last_joint_state.position[2];
		joint_msg.joints[3]       = last_joint_state.position[3];
		joint_msg.joints[4]       = last_joint_state.position[4];
		joint_msg.joints[5]       = last_joint_state.position[5];
		joint_msg.jaw = std::abs(last_gripper_pos.data - gripper_open_pos) /
		                std::abs(gripper_closed_pos - gripper_open_pos);
		joint_pos_pub->publish(joint_msg);
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::Arm1MoveitCompatNode)
