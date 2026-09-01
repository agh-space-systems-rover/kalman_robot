#include "kalman_arm_controller/arm_hardware.hpp"
#include "kalman_arm_controller/can_libs/arm_config.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"

namespace kalman_arm_controller {

const double      MAX_POS_DIFF       = 0.1;
const std::string CONTROL_TYPE_TOPIC = "/change_control_type";

class ArmSimSystem : public ArmSystem {
public:

hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) !=
	    CallbackReturn::SUCCESS) {
		return CallbackReturn::ERROR;
	}


    arm_config::load_default_config();
    joint_position_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);

    for (const auto &joint : info_.joints) {
                for (const auto &interface : joint.state_interfaces) {
                    joint_interfaces[interface.name].push_back(joint.name);
                }
            }
    node_ = rclcpp::Node::make_shared("arm_sim_hardware_node");

    control_type_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
        CONTROL_TYPE_TOPIC, 10, [&](std_msgs::msg::UInt8::SharedPtr msg) {
            switch (msg->data) {
            case 0:
                current_control_type = ControlType::position;
                break;
            case 1:
                current_control_type = ControlType::posvel;
                break;
            }
        }
    );
    node_future_ = std::async([&]() {
        rclcpp::spin(node_);
    });
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // The state is already instantly updated in write().
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    bool pos_too_far = false;
        for (int i = 0; i < 6; i++) {
            if (abs(joint_position_command_[i] - joint_position_[i]) >
                MAX_POS_DIFF) {
                pos_too_far = true;
                break;
            }
        }
    if (current_control_type == ControlType::posvel || (current_control_type == ControlType::position && !pos_too_far && already_read_)){
        for (int i = 0; i < 4; i++) {
            CAN_vars::joints[i].moveSetpoint.position_deg =
                joint_position_command_[i] * 180.0f / M_PI;
            CAN_vars::joints[i].moveSetpoint.velocity_deg_s =
                joint_velocities_command_[i] * 180.0f / M_PI;
            CAN_vars::joints[i].moveSetpoint.torque_Nm = 0x02fa;
            CAN_vars::joints[i].moveSetpoint.acceleration_deg_ss =
                0xffff;
        }
        for (int i = 4; i < 6; i++) {
            CAN_vars::joints[i].moveSetpointDiff.position_deg =
                joint_position_command_[i] * 180.0f / M_PI;
            CAN_vars::joints[i].moveSetpointDiff.velocity_deg_s =
                joint_velocities_command_[i] * 180.0f / M_PI;
            CAN_vars::joints[i].moveSetpointDiff.torque_Nm = 0x02fa;
            CAN_vars::joints[i].moveSetpointDiff.acceleration_deg_ss =
                0xffff;
        }
        float temp_pos = CAN_vars::joints[0].moveSetpoint.position_deg;
        float temp_vel = CAN_vars::joints[0].moveSetpoint.velocity_deg_s;
        CAN_vars::joints[0].moveSetpoint.position_deg   = -temp_pos;
        CAN_vars::joints[0].moveSetpoint.velocity_deg_s = -temp_vel;
        
    }
    for (int i = 0; i < 6; i++) {
            joint_position_[i] = joint_position_command_[i];
            joint_velocities_[i] = joint_velocities_command_[i];
        }
    return hardware_interface::return_type::OK;
}

};

} // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kalman_arm_controller::ArmSimSystem, hardware_interface::SystemInterface)