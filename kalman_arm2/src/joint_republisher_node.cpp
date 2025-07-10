#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <functional>

#include <kalman_interfaces/msg/arm_values.hpp>

namespace kalman_arm2 {

class JointRepublisher : public rclcpp::Node {
  public:
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr sub;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;

	JointRepublisher(const rclcpp::NodeOptions &options)
	    : Node("joint_republisher", options) {
		
		// Create publisher for joint states
		pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
		
		// Subscribe to joint status
		sub = create_subscription<kalman_interfaces::msg::ArmValues>(
			"current_pos", 10,
			std::bind(&JointRepublisher::on_arm_values, this, std::placeholders::_1));
	}

private:
	void on_arm_values(const kalman_interfaces::msg::ArmValues::SharedPtr msg) {
		auto joint_state = sensor_msgs::msg::JointState();
		joint_state.header.stamp = now();
		
		// Joint names for 6-DOF arm + jaw
		joint_state.name = {
			"arm_joint_1", "arm_joint_2", "arm_joint_3", 
			"arm_joint_4", "arm_joint_5", "arm_joint_6", "arm_joint_jaw"
		};
		
		// Copy joint positions
		joint_state.position.resize(7);
		for (size_t i = 0; i < 6; ++i) {
			joint_state.position[i] = msg->joints[i];
		}
		joint_state.position[6] = msg->jaw;
		
		pub->publish(joint_state);
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::JointRepublisher)
