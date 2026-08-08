#pragma once
#include "kalman_arm_controller/can_libs/can_driver.hpp"
#include "kalman_arm_controller/can_libs/can_handlers.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class CanDriver {
	using DriverVars_t                = CAN_driver::DriverVars_t;
	static constexpr auto BUFFER_SIZE = 1024;

	/**
	 * @brief Initialize the CAN driver.
	 *
	 * Initializes the CAN driver and binds the socket to the can0 interface
	 *
	 * @return int 0 on success, 1 on failure
	 */
	int init(const char *can_interface);

	int startArmRead();

	int armRead();

	/**
	 * @brief Handle a received frame.
	 *
	 * Decodes the frame and calls the appropriate handler function.
	 *
	 * @param frame The frame to handle
	 * @return int 0 on success, 1 on failure
	 */
	int handle_frame(
	    canfd_frame                                         frame,
	    const std::unordered_map<uint8_t, canCmdHandler_t> *handles
	);

	int arm_write(ControlType controlType);

	int write_control_type(ControlType controlType);

	int write_joint_setpoint(uint8_t joint_id);

	int write_joint_posvel(uint8_t joint_id);

	int write_gripper_position(uint16_t position);

	int write_fastclick(uint8_t position);

	int write_data(uint16_t can_id, uint8_t *data, uint8_t len);

	template <typename T> int write_data(uint16_t can_id, const T &data) {
		static_assert(
		    std::is_trivial<T>::value, "Input must be a trivial datatype"
		);
		static_assert(
		    std::is_standard_layout<T>::value, "Input must be standard layout"
		);
		struct canfd_frame frame;
		frame.can_id = can_id;
		static_assert(
		    sizeof(T) <= std::numeric_limits<decltype(frame.len)>::max(),
		    "T size must fit in frame.len data type"
		);
		frame.len   = sizeof(T);
		frame.flags = 0;
		memcpy(frame.data, &data, frame.len);
		if (::write(driver_vars.sock, &frame, sizeof(frame)) < 0) {
			perror("Write");
			return 1;
		}
		return 0;
	}

	int close();

private:
	DriverVars_t driver_vars{};
};
