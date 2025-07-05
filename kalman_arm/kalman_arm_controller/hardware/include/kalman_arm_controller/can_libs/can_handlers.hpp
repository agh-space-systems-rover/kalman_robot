#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP

#include "can_types.hpp"
#include <unordered_map>

namespace CAN_handlers
{
// For each command, define a handler function
int handle_joint_status(uint32_t identifier, uint8_t* data, uint8_t len);
int handle_joint_fast_status(uint32_t identifier, uint8_t* data, uint8_t len);
extern "C" {
int handle_gripper_status(uint32_t identifier, uint8_t* data, uint8_t len);
};

// Declare the command handler array
extern const std::unordered_map<uint8_t, canCmdHandler_t> HANDLES;
}  // namespace CAN_handlers

#endif  // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP