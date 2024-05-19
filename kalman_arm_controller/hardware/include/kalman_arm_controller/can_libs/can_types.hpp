/**
 * @file can_types.hpp
 * @brief Contains the definitions of CAN types used in the Kalman Arm Controller hardware.
 */

#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_

#include <cstdint>
#include "can_messages.hpp"

/**
 * @brief Structure representing a CAN message handler.
 *
 * This structure is used to store the CAN message handler functions.
 *
 * @param can_id uint16_t Command identifier
 * @param len uint8_t Length of the data
 * @param func Function pointer to the handler
 */
typedef struct
{
    uint16_t can_id;
    uint8_t len;
    int (*func)(uint32_t identifier, uint8_t *data, uint8_t len);
} canCmdHandler_t;

/**
 * @brief Structure representing the status of a joint motor laready calculated to normal, humanreadable
 * and supported by moveit format.
 */
typedef struct
{
    float velocity_deg_s;
    float position_deg;
} jointMoveStatus_t;

typedef struct
{
    float torque_Nm;
    float velocity_deg_s;
    float position_deg;
    float acceleration_deg_ss;
} jointMoveSetpoint_t;

/**
 * @brief Structure representing the status of a joint motor and its setpoint.
 *
 * This structure combines the joint motor status and the joint setpoint.
 *
 * @param status jointMotorStatus_t Received joint motor status
 * @param setpoint jointCmdSetpoint_t Joint setpoint to send
 */
typedef struct __attribute__((__packed__))
{
    /**
     * @brief Structure representing the status of a joint motor received from CAN.
     */
    jointMotorStatus_t status;

    /**
     * @brief Structure representing the fast status (only pos and vel) of a joint motor received from CAN.
     */
    jointMotorFastStatus_t fastStatus;

    /**
     * @brief Structure representing the setpoint of a joint motor to send via CAN.
     */
    jointCmdSetpoint_t setpoint;

    jointCmdVelocity_t velSetpoint;

    /**
     * @brief Structure representing the received status of a joint motor already calculated to normal, humanreadable
     * and supported by moveit format.
     */
    jointMoveStatus_t moveStatus;

    /**
     * @brief Structure representing the setpoint of a joint motor already calculated to normal, humanreadable
     * and supported by moveit format.
     */
    jointMoveSetpoint_t moveSetpoint;

    /**
     * @brief Structure where the setpoint of differential joints is stored (later converted to `moveSetpoint`).
     */
    jointMoveSetpoint_t moveSetpointDiff;
} jointStatus_t;

/**
 * @brief Structure representing the configuration of a joint motor.
 *
 * This structure is used to store the configuration of a joint motors in arm_config file.
 */
typedef struct
{

    float maxVelocity_deg_s;
    float maxAcceleration_deg_ss;
    float maxTorque_Nm;

    float minPosition_deg, maxPosition_deg;

    float positionAfterPositioning;
    float idleTorque_Nm;
    float defVelocity_deg_s;
    float defAcceleration_deg_ss;
    float defAcceleration_pos_deg_ss;

    float gearRatio;

    uint8_t invertDirection;
    uint8_t requirePositioning;
    uint8_t positioningOrder;
    float positioningVelocity;
    uint16_t positioningTimeout;
    uint8_t differential;

} jointConfig_t;

/**
 * @brief Structure representing the configuration of the arm.
 *
 * This structure is used to store the configuration of the arm (every joint) in arm_config file.
 */
typedef struct
{
    uint8_t jointNumber;

    jointConfig_t joint[16];
    uint16_t jointCommandRefreshTime_ms;
    uint16_t jointCommunicationTimeout;
    uint16_t canRoverStatusSendPeriod_ms;

} armConfig_t;

enum ControlType
{
    position,
    posvel
};

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_