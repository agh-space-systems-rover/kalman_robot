#include "kalman_arm_controller/arm_hardware.hpp"

#include <cmath>

namespace kalman_arm_controller {

class ArmSimSystem : public ArmSystem {
public:
	CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
		if (hardware_interface::SystemInterface::on_init(info) !=
		    CallbackReturn::SUCCESS) {
			return CallbackReturn::ERROR;
		}

		const size_t joint_count = info_.joints.size();
		if (joint_count != 6 && joint_count != 7) {
			return CallbackReturn::ERROR;
		}
		for (size_t i = 0; i < 6; ++i) {
			if (info_.joints[i].name != "arm_joint_" + std::to_string(i + 1)) {
				return CallbackReturn::ERROR;
			}
		}
		if (joint_count == 7 && info_.joints[6].name != "arm_joint_jaw") {
			return CallbackReturn::ERROR;
		}
		joint_position_.assign(joint_count, 0.0);
		joint_velocities_.assign(joint_count, 0.0);
		joint_position_command_.assign(joint_count, 0.0);
		joint_velocities_command_.assign(joint_count, 0.0);

		for (const auto &joint : info_.joints) {
			for (const auto &interface : joint.state_interfaces) {
				joint_interfaces[interface.name].push_back(joint.name);
			}
		}

		node_ = rclcpp::Node::make_shared("arm_sim_hardware_node");
		control_type_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
		    "/change_control_type", 10,
		    [this](std_msgs::msg::UInt8::SharedPtr msg) {
			    switch (msg->data) {
			    case 0:
				    current_control_type = ControlType::position;
				    break;
			    case 1:
				    current_control_type = ControlType::posvel;
				    break;
			    default:
				    RCLCPP_ERROR(node_->get_logger(), "Invalid control type: %d", msg->data);
			    }
		    }
		);
		return CallbackReturn::SUCCESS;
	}

	return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
		// Process mode changes on the control thread, avoiding a detached spinner
		// and concurrent access to current_control_type.
		rclcpp::spin_some(node_);
		return return_type::OK;
	}

	return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override {
		const double dt = period.seconds();
		if (!std::isfinite(dt) || dt < 0.0) {
			return return_type::ERROR;
		}
		for (size_t i = 0; i < joint_position_.size(); ++i) {
			if (!std::isfinite(joint_velocities_command_[i]) ||
			    (current_control_type == ControlType::position &&
			     !std::isfinite(joint_position_command_[i]))) {
				return return_type::ERROR;
			}
		}

		// This ideal simulation stays in ROS joint coordinates; it must not
		// mutate shared CAN commands or apply motor/differential conversions.
		for (size_t i = 0; i < joint_position_.size(); ++i) {
			joint_velocities_[i] = joint_velocities_command_[i];
			if (current_control_type == ControlType::posvel) {
				joint_position_[i] += joint_velocities_[i] * dt;
			} else {
				joint_position_[i] = joint_position_command_[i];
			}
		}
		return return_type::OK;
	}
};

} // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    kalman_arm_controller::ArmSimSystem, hardware_interface::SystemInterface
)
