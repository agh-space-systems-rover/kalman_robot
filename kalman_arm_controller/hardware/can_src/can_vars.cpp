#include "kalman_arm_controller/can_libs/can_vars.hpp"

#include <stdio.h>

namespace CAN_vars
{
jointStatus_t joints[6] = {};

armConfig_t arm_config = {};

bool received_joint_status[6] = { false, false, false, false, false, false };
}  // namespace CAN_vars

/**
 * @brief Update joint setpoint from global joints - converts from CAN
 * message to human readable units.
 */
void CAN_vars::update_joint_status()
{
  for (uint8_t i = 0; i < arm_config.jointNumber; i++)
  {
    update_single_joint_status(i);
  }
}

/**
 * @brief Update joint setpoint from global joints - converts from human readable units to
 * CAN message types. Also calculates differential joint setpoints.
 */
void CAN_vars::update_joint_setpoint()
{
  calculate_diff_setpoints();
  for (uint8_t i = 0; i < arm_config.jointNumber; i++)
  {
    update_single_joint_setpoint(i);
  }
}

/**
 * @brief Update joint setpoint from global joints depending on differential joint
 *
 * @param joint_id - from 0 to 5
 */
void CAN_vars::update_single_joint_status(uint8_t joint_id)
{
  jointConfig_t* config = &CAN_vars::arm_config.joint[joint_id + 1];
  if (config->differential)
  {
    calculate_status_diff(joint_id, config->differential);
  }
  else
  {
    calculate_status(joint_id);
  }
}

/**
 * @brief Update joint setpoint from global joints - converting to human readable units
 *
 * @param joint_id - from 0 to 5
 */
void CAN_vars::calculate_status(uint8_t joint_id)
{
  jointConfig_t* config = &CAN_vars::arm_config.joint[joint_id + 1];
  jointMotorFastStatus_t* jointStatus = &CAN_vars::joints[joint_id].fastStatus;
  float gearRatio = config->gearRatio;
  int direction = config->invertDirection ? -1 : 1;

  joints[joint_id].moveStatus.velocity_deg_s = (360.0f / (10 * 60)) * gearRatio * jointStatus->velocity * direction;
  joints[joint_id].moveStatus.position_deg = 0.01f * gearRatio * jointStatus->position * direction;
}

/**
 * @brief Calculate differential joint status -  into human readable units
 *
 * @param joint_id - from 0 to 5
 * @param diff_id - differential joint id (from 1 to 6)
 */
void CAN_vars::calculate_status_diff(uint8_t joint_id, uint8_t diff_id)
{
  uint8_t difNbr[2];
  jointConfig_t* difConfig[2];
  jointMotorFastStatus_t* difStatus[2];

  float gearRatio[2], velocity[2], position[2];

  if (diff_id > joint_id + 1)
  {
    difNbr[0] = joint_id + 1;
    difNbr[1] = diff_id;
  }
  else
  {
    difNbr[0] = diff_id;
    difNbr[1] = joint_id + 1;
  }

  difConfig[0] = &CAN_vars::arm_config.joint[difNbr[0]];
  difConfig[1] = &CAN_vars::arm_config.joint[difNbr[1]];

  difStatus[0] = &CAN_vars::joints[difNbr[0] - 1].fastStatus;
  difStatus[1] = &CAN_vars::joints[difNbr[1] - 1].fastStatus;

  for (uint8_t i = 0; i < 2; i++)
  {
    gearRatio[i] = difConfig[i]->gearRatio;
    velocity[i] = (360.0f / (10 * 60)) * gearRatio[i] * difStatus[i]->velocity;
    position[i] = 0.01f * gearRatio[i] * difStatus[i]->position;
  }

  joints[difNbr[0] - 1].moveStatus.velocity_deg_s = (velocity[0] - velocity[1]) / 2;
  joints[difNbr[0] - 1].moveStatus.position_deg = -(position[0] - position[1]) / 2;

  joints[difNbr[1] - 1].moveStatus.velocity_deg_s = (velocity[0] + velocity[1]) / 2;
  joints[difNbr[1] - 1].moveStatus.position_deg = (position[0] + position[1]) / 2;
}

/**
 * @brief Update joint setpoint from global joints - converting to CAN message types
 *
 * @param joint_id - from 0 to 5
 */
void CAN_vars::update_single_joint_setpoint(uint8_t joint_id)
{
  jointCmdSetpoint_t data;
  int direction;
  float gearRatio, temp;

  direction = arm_config.joint[joint_id + 1].invertDirection ? -1 : 1;
  gearRatio = arm_config.joint[joint_id + 1].gearRatio;

  temp = (100.0f / gearRatio) * joints[joint_id].moveSetpoint.position_deg * direction;
  if (temp >= INT32_MAX)
    data.position_0deg01 = INT32_MAX;
  else if (temp <= INT32_MIN)
    data.position_0deg01 = INT32_MIN;
  else
    data.position_0deg01 = temp;

  // if(joint_id==0)
  //     printf("GOT setpoint in degs: %f\t setting setpoint to: %ld\t\r\n", joints[joint_id].moveSetpoint.position_deg,
  //     data.position_0deg01);

  CAN_vars::joints[joint_id].setpoint = data;

  jointCmdVelocity_t velData;

  temp = (((10.0f * 60) / 360.0f) / gearRatio) * joints[joint_id].moveSetpoint.velocity_deg_s * direction;
  if (temp >= INT16_MAX)
    velData.velocity_0RPM_1 = INT16_MAX;
  else if (temp <= INT16_MIN)
    velData.velocity_0RPM_1 = INT16_MIN;
  else
    velData.velocity_0RPM_1 = temp;

  CAN_vars::joints[joint_id].velSetpoint = velData;
}

/**
 * @brief Calculate differential joint setpoints if needed
 */
void CAN_vars::calculate_diff_setpoints()
{
  for (uint8_t i = 0; i < arm_config.jointNumber; i++)
  {
    jointConfig_t* config = &CAN_vars::arm_config.joint[i + 1];
    if (config->differential > i + 1)
    {
      calculate_single_diff_setpoint(i, config->differential - 1);
    }
  }
}

/**
 * @brief Calculate differential joint setpoints
 *
 * @param joint_id - from 0 to 5
 * @param diff_id - differential joint id (from 0 to 5)
 */
void CAN_vars::calculate_single_diff_setpoint(uint8_t joint_id, uint8_t diff_id)
{
  float dif1Velocity, dif2Velocity;
  float dif1Position, dif2Position;

  dif1Position = joints[joint_id].moveSetpointDiff.position_deg;
  dif2Position = joints[diff_id].moveSetpointDiff.position_deg;

  dif1Velocity = joints[joint_id].moveSetpointDiff.velocity_deg_s;
  dif2Velocity = joints[diff_id].moveSetpointDiff.velocity_deg_s;

  joints[joint_id].moveSetpoint.torque_Nm = joints[joint_id].moveSetpointDiff.torque_Nm;
  joints[diff_id].moveSetpoint.torque_Nm = joints[diff_id].moveSetpointDiff.torque_Nm;

  joints[joint_id].moveSetpoint.acceleration_deg_ss = joints[joint_id].moveSetpointDiff.acceleration_deg_ss;
  joints[diff_id].moveSetpoint.acceleration_deg_ss = joints[diff_id].moveSetpointDiff.acceleration_deg_ss;

  joints[joint_id].moveSetpoint.velocity_deg_s = -dif1Velocity + dif2Velocity;
  joints[diff_id].moveSetpoint.velocity_deg_s = dif1Velocity + dif2Velocity;

  joints[joint_id].moveSetpoint.position_deg = -dif1Position + dif2Position;
  joints[diff_id].moveSetpoint.position_deg = dif1Position + dif2Position;
}
