#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP

#include "can_types.hpp"
#include "arm_config.hpp"
#define ABS_F(x) (x >= 0 ? (x) : ((x) * (-1.0f)))

namespace CAN_vars
{
/**
 * @brief Array of jointStatus_t representing the status of each joint.
 * This includes status, setpoint in CAN and readable units. Joint numbered from 0 to 5.
 */
extern jointStatus_t joints[6];

/**
 * @brief armConfig_t representing the configuration of the arm.
 * This includes the number of joints and the configuration of each joint.
 *
 * NOTE! Joint numbering starts from 1, so joint[0] is not used.
 */
extern armConfig_t arm_config;

extern bool received_joint_status[6];

void update_joint_status();
void update_joint_setpoint();

void update_single_joint_status(uint8_t joint_id);
void calculate_status(uint8_t joint_id);
void calculate_status_diff(uint8_t joint_id, uint8_t diff_id);
void update_single_joint_setpoint(uint8_t joint_id);
void calculate_diff_setpoints();
void calculate_single_diff_setpoint(uint8_t joint_id, uint8_t diff_id);
}  // namespace CAN_vars

#endif  // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP
