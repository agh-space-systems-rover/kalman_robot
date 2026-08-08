#include "kalman_arm_controller/arm_hardware.hpp"
#include "hardware_interface/system_interface.hpp"
#include "kalman_arm_controller/can_libs/can_driver.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"
#include "kalman_arm_controller/can_libs/new_can_driver.hpp"
#include <future>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>

const double      MAX_POS_DIFF       = 0.1;
const char*       CAN_INTERFACE      = "can0";
const std::string CONTROL_TYPE_TOPIC = "/change_control_type";

namespace kalman_arm_controller {
CallbackReturn
ArmSystem::on_init(const hardware_interface::HardwareInfo &info) {
	if (hardware_interface::SystemInterface::on_init(info) !=
	    CallbackReturn::SUCCESS) {
		return CallbackReturn::ERROR;
	}

	// robot has 6 joints and 2 interfaces
	joint_position_.assign(6, 0);
	joint_velocities_.assign(6, 0);
	joint_position_command_.assign(6, 0);
	joint_velocities_command_.assign(6, 0);

	for (const auto &joint : info_.joints) {
		for (const auto &interface : joint.state_interfaces) {
			joint_interfaces[interface.name].push_back(joint.name);
		}
	}

	can_driver.init(CAN_INTERFACE);
	can_driver.startArmRead();

	node_ = rclcpp::Node::make_shared("arm_hardware_node");

	control_type_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
	    CONTROL_TYPE_TOPIC, 10, [&](std_msgs::msg::UInt8::SharedPtr msg) {
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

	node_future_ = std::async([&]() {
		rclcpp::spin(node_);
	});

	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmSystem::export_state_interfaces() {
	std::vector<hardware_interface::StateInterface> state_interfaces;

	size_t ind = 0;
	for (const auto &joint_name : joint_interfaces["position"]) {
		state_interfaces.emplace_back(
		    joint_name, "position", &joint_position_[ind++]
		);
	}

	ind = 0;
	for (const auto &joint_name : joint_interfaces["velocity"]) {
		state_interfaces.emplace_back(
		    joint_name, "velocity", &joint_velocities_[ind++]
		);
	}

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmSystem::export_command_interfaces() {
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	size_t ind = 0;
	for (const auto &joint_name : joint_interfaces["position"]) {
		command_interfaces.emplace_back(
		    joint_name, "position", &joint_position_command_[ind++]
		);
	}

	ind = 0;
	for (const auto &joint_name : joint_interfaces["velocity"]) {
		command_interfaces.emplace_back(
		    joint_name, "velocity", &joint_velocities_command_[ind++]
		);
	}

	return command_interfaces;
}

return_type
ArmSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */) {
	return read_joint_states();
	// if (current_control_type == ControlType::posvel)
	// {
	//     joint_velocities_ = joint_velocities_command_;
	//     for (int i = 0; i < 6; i++)
	//     {
	//         joint_position_[i] += joint_velocities_[i] * period.seconds();
	//     }
	// }
	// else
	// {
	//     joint_position_ = joint_position_command_;
	//     joint_velocities_ = joint_velocities_command_;
	// }

	// return return_type::OK;
}

return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
	return write_joint_commands();
	// return return_type::OK;
}

return_type ArmSystem::read_joint_states() {
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	already_read_ = true;
	for (size_t i = 0; i < 6; i++) {
		joint_position_[i] =
		    CAN_vars::joints.jointFeedback[i].moveStatus.position_deg * M_PI / 180.0f;
		joint_velocities_[i] =
		    CAN_vars::joints.jointFeedback[i].moveStatus.velocity_deg_s * M_PI / 180.0f;

		if (!CAN_vars::joints.jointFeedback[i].received) {
			already_read_ = false;
		}
	}

	// Flip joint 1 (base)
	joint_position_[0]   = -joint_position_[0];
	joint_velocities_[0] = -joint_velocities_[0];

	return return_type::OK;
}

return_type ArmSystem::write_joint_commands() {
	// Do not write if previous write is still in progress
	if (writer.valid() &&
	    writer.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
		// RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Previous write still in
		// progress");
	} else {
		{
			bool pos_too_far = false;
			for (size_t i = 0; i < 6; i++) {
				if (std::abs(joint_position_command_[i] - joint_position_[i]) >
				    MAX_POS_DIFF) {
					pos_too_far = true;
					break;
				}
			}

			if (current_control_type == ControlType::posvel ||
			    (current_control_type == ControlType::position &&
			     !pos_too_far && already_read_)) {
				std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
				for (size_t i = 0; i < 4; i++) {
					CAN_vars::joints.jointCmd[i].moveSetpoint.position_deg =
					    joint_position_command_[i] * 180.0f / M_PI;
					CAN_vars::joints.jointCmd[i].moveSetpoint.velocity_deg_s =
					    joint_velocities_command_[i] * 180.0f / M_PI;
					CAN_vars::joints.jointCmd[i].moveSetpoint.torque_Nm = 0x02fa;
					CAN_vars::joints.jointCmd[i].moveSetpoint.acceleration_deg_ss =
					    0xffff;
				}
				for (int i = 4; i < 6; i++) {
					CAN_vars::joints.jointCmd[i].moveSetpointDiff.position_deg =
					    joint_position_command_[i] * 180.0f / M_PI;
					CAN_vars::joints.jointCmd[i].moveSetpointDiff.velocity_deg_s =
					    joint_velocities_command_[i] * 180.0f / M_PI;
					CAN_vars::joints.jointCmd[i].moveSetpointDiff.torque_Nm = 0x02fa;
					CAN_vars::joints.jointCmd[i].moveSetpointDiff.acceleration_deg_ss =
					    0xffff;
				}

				// Flip joint 1 (base)
				float temp_pos = CAN_vars::joints.jointCmd[0].moveSetpoint.position_deg;
				float temp_vel =
				    CAN_vars::joints.jointCmd[0].moveSetpoint.velocity_deg_s;
				CAN_vars::joints.jointCmd[0].moveSetpoint.position_deg   = -temp_pos;
				CAN_vars::joints.jointCmd[0].moveSetpoint.velocity_deg_s = -temp_vel;

				// Run write in a separate thread
				writer = std::async(std::launch::async, [&] {
					// CAN_driver::arm_write(current_control_type);
					can_driver.arm_write(current_control_type);
				});
			}
		}
	}

	return return_type::OK;
}

ArmSystem::~ArmSystem() {
	can_driver.close();
}

} // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kalman_arm_controller::ArmSystem, hardware_interface::SystemInterface
)
