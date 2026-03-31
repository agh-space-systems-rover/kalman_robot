#ifndef KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_
#define KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "can_libs/can_types.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace kalman_arm_controller {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ArmSystem
    : public hardware_interface::SystemInterface {
  public:
    using SystemInterface::SystemInterface;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

	std::vector<hardware_interface::StateInterface>
   
	export_state_interfaces() override;

	std::vector<hardware_interface::CommandInterface>
   
	export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	hardware_interface::return_type
    write(
	    const rclcpp::Time  &time, const rclcpp::Duration  &period
	) override;

	~ArmSystem() override;

  private:
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
        control_type_subscriber_;
    std::future<void> node_future_;
    std::future<void> writer;

    ControlType current_control_type{ControlType::posvel};
    bool already_read_{false};

    hardware_interface::return_type read_joint_states();
    hardware_interface::return_type write_joint_commands();
};

} // namespace kalman_arm_controller

#endif // KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_
