#include "kalman_arm_controller/arm_hardware.hpp"
#include "kalman_arm_controller/can_libs/can_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>
// #include "arm_hardware.hpp"

const double MAX_POS_DIFF = 0.1;

namespace kalman_arm_controller
{
CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto& joint : info_.joints)
  {
    for (const auto& interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  CAN_driver::init(&CAN_driver::arm_driver, "can0");
  CAN_driver::startArmRead();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto& joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto& joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto& joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto& joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type ArmSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  return read_joint_states();
}

return_type ArmSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  return write_joint_commands();
}

return_type ArmSystem::read_joint_states()
{
  // CAN_driver::read();
  std::lock_guard<std::mutex> lock(CAN_driver::arm_driver.m_read);
  // RCLCPP_INFO(rclcpp::get_logger("my_logger"), "position: %ld",
  // CAN_vars::joints[0].status.position);
  already_read_ = true;
  for (int i = 0; i < 6; i++)
  {
    joint_position_[i] = CAN_vars::joints[i].moveStatus.position_deg * M_PI / 180.0f;
    joint_velocities_[i] = CAN_vars::joints[i].moveStatus.velocity_deg_s * M_PI / 180.0f;

    if (!CAN_vars::received_joint_status[i])
      already_read_ = false;
  }
  return return_type::OK;
}

return_type ArmSystem::write_joint_commands()
{
  // Do not write if previous write is still in progress
  if (writer.valid() && writer.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    // RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Previous write still in
    // progress");
  }
  else
  {
    {
      bool pos_too_far = false;
      for (int i = 0; i < 6; i++)
      {
        if (abs(joint_position_command_[i] - joint_position_[i]) > MAX_POS_DIFF)
        {
          pos_too_far = true;
          break;
        }
      }

      if (current_control_type == ControlType::posvel ||
          (current_control_type == ControlType::position && !pos_too_far && already_read_))
      {
        std::lock_guard<std::mutex> lock(CAN_driver::arm_driver.m_write);
        for (int i = 0; i < 4; i++)
        {
          CAN_vars::joints[i].moveSetpoint.position_deg = joint_position_command_[i] * 180.0f / M_PI;
          CAN_vars::joints[i].moveSetpoint.velocity_deg_s = joint_velocities_command_[i] * 180.0f / M_PI;
          CAN_vars::joints[i].moveSetpoint.torque_Nm = 0x02fa;
          CAN_vars::joints[i].moveSetpoint.acceleration_deg_ss = 0xffff;
        }
        for (int i = 4; i < 6; i++)
        {
          CAN_vars::joints[i].moveSetpointDiff.position_deg = joint_position_command_[i] * 180.0f / M_PI;
          CAN_vars::joints[i].moveSetpointDiff.velocity_deg_s = joint_velocities_command_[i] * 180.0f / M_PI;
          CAN_vars::joints[i].moveSetpointDiff.torque_Nm = 0x02fa;
          CAN_vars::joints[i].moveSetpointDiff.acceleration_deg_ss = 0xffff;
        }
      }
      // Run write in a separate thread
      writer = std::async(std::launch::async, [&] { CAN_driver::arm_write(current_control_type); });
    }
  }

  return return_type::OK;
}

}  // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kalman_arm_controller::ArmSystem, hardware_interface::SystemInterface)
