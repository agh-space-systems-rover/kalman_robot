#include "kalman_arm_controller/can_libs/can_handlers.hpp"
#include "kalman_arm_controller/can_libs/can_messages.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"
#include <cstring>
#include <linux/can.h>

namespace {
bool valid_joint_frame(uint32_t identifier, uint8_t command) {
	const uint32_t joint_id = identifier >> 7;
	return (identifier & ~CAN_SFF_MASK) == 0 &&
	       (identifier & 0x7fU) == command && joint_id >= 1 && joint_id <= 6;
}
} // namespace

namespace CAN_handlers {
const std::unordered_map<uint8_t, canCmdHandler_t> HANDLES = {
    {CMD_JOINT_STATUS,
     {CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status}},
    {CMD_JOINT_FAST_STATUS,
     {CMD_JOINT_FAST_STATUS,
      sizeof(jointMotorFastStatus_t),
      handle_joint_fast_status}}
};

int handle_joint_status(uint32_t identifier, uint8_t *data, uint8_t len) {
	if (!valid_joint_frame(identifier, CMD_JOINT_STATUS) ||
	    data == nullptr || len != sizeof(jointMotorStatus_t)) {
		return 1;
	}
	const uint32_t joint_id = (identifier >> 7) - 1;
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	std::memcpy(&CAN_vars::joints.jointFeedback[joint_id].status, data,
	            sizeof(jointMotorStatus_t));
	return 0;
}

int handle_joint_fast_status(uint32_t identifier, uint8_t *data, uint8_t len) {
	if (!valid_joint_frame(identifier, CMD_JOINT_FAST_STATUS) ||
	    data == nullptr || len != sizeof(jointMotorFastStatus_t)) {
		return 1;
	}
	const uint32_t joint_id = (identifier >> 7) - 1;
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	std::memcpy(&CAN_vars::joints.jointFeedback[joint_id].fastStatus, data,
	            sizeof(jointMotorFastStatus_t));
	// Publish raw feedback, both differential outputs, and readiness together.
	CAN_vars::update_joint_status_locked();
	CAN_vars::joints.jointFeedback[joint_id].received = true;
	return 0;
}

int handle_gripper_status(uint32_t identifier, uint8_t *data, uint8_t len) {
	if (identifier != CMD_GET_GRIPPER || data == nullptr ||
	    len != sizeof(uint16_t)) {
		return 1;
	}
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	std::memcpy(&CAN_vars::joints.gripper_position, data, sizeof(uint16_t));
	return 0;
}

} // namespace CAN_handlers
