#include "kalman_arm_controller/can_libs/can_handlers.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"
#include "kalman_arm_controller/can_libs/can_messages.hpp"

#define UNUSED(x) (void)(x)

namespace CAN_handlers
{
// Define the command handler array
const std::unordered_map<uint8_t, canCmdHandler_t> HANDLES = {
    { CMD_JOINT_STATUS, { CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status } },
    { CMD_JOINT_FAST_STATUS,
      { CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t), handle_joint_fast_status } }
};

/**
 * @brief Handle the joint status command.
 *
 * Updates the global joint status.
 */
int handle_joint_status(uint32_t identifier, uint8_t* data, uint8_t len)
{
    UNUSED(len);

    uint8_t joint_id = identifier >> 7;
    // uint8_t command = identifier - (joint_id << 7);

    joint_id--;

    const jointMotorStatus_t* status = (jointMotorStatus_t*)data;

    // Update the joint status
    CAN_vars::joints[joint_id].status = *status;
    return 0;
}

/**
 * @brief Handle the joint fast status (pos and vel) command.
 *
 * Updates the global joint fast status.
 */
int handle_joint_fast_status(uint32_t identifier, uint8_t* data, uint8_t len)
{
    UNUSED(len);

    uint8_t joint_id = identifier >> 7;
    // uint8_t command = identifier - (joint_id << 7);

    joint_id--;

    const jointMotorFastStatus_t* status = (jointMotorFastStatus_t*)data;

    CAN_vars::joints[joint_id].fastStatus = *status;
    CAN_vars::received_joint_status[joint_id] = true;
    return 0;
}

int handle_gripper_status(uint32_t identifier, uint8_t* data, uint8_t len)
{
    UNUSED(identifier);
    UNUSED(len);

    CAN_vars::gripper_position = *((uint16_t*)data);
    return 0;
}

}  // namespace CAN_handlers