/**
 * @file can_types.hpp
 * @brief Contains the definitions of CAN types used in the Kalman Arm
 * Controller hardware.
 */

#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_

#include "can_messages.hpp"
#include <array>
#include <cstdint>
#include <mutex>

/**
 * @brief Structure representing a CAN message handler.
 *
 * This structure is used to store the CAN message handler functions.
 *
 * @param can_id uint16_t Command identifier
 * @param len uint8_t Length of the data
 * @param func Function pointer to the handler
 */
struct canCmdHandler_t{
	uint16_t can_id;
	uint8_t  len;
	int (*func)(uint32_t identifier, uint8_t *data, uint8_t len);
};

/**
 * @brief Structure representing the status of a joint motor laready calculated
 * to normal, humanreadable and supported by moveit format.
 */
struct jointMoveStatus_t {
	float velocity_deg_s;
	float position_deg;
};

struct jointMoveSetpoint_t {
	float torque_Nm;
	float velocity_deg_s;
	float position_deg;
	float acceleration_deg_ss;
};

/**
 * @brief Structure representing the status of a joint motor.
 *
 * @param status jointMotorStatus_t Received joint motor status
 */
struct JointFeedback {
    /**
	 * @brief Structure representing the status of a joint motor received from
	 * CAN.
	 */
	jointMotorStatus_t status;

	/**
	 * @brief Structure representing the fast status (only pos and vel) of a
	 * joint motor received from CAN.
	 */
	jointMotorFastStatus_t fastStatus;

	/**
	 * @brief Structure representing the received status of a joint motor
	 * already calculated to normal, humanreadable and supported by moveit
	 * format.
	 */
	jointMoveStatus_t moveStatus;

	bool received{};

};

struct JointCommand {
    /**
	 * @brief Structure representing the setpoint of a joint motor to send via
	 * CAN.
	 */
	jointCmdSetpoint_t setpoint;

	jointCmdVelocity_t velSetpoint;

	/**
	 * @brief Structure representing the setpoint of a joint motor already
	 * calculated to normal, humanreadable and supported by moveit format.
	 */
	jointMoveSetpoint_t moveSetpoint;

	/**
	 * @brief Structure where the setpoint of differential joints is stored
	 * (later converted to `moveSetpoint`).
	 */
	jointMoveSetpoint_t moveSetpointDiff;
};

struct ArmState {
    std::array<JointFeedback, 6> jointFeedback;
    std::array<JointCommand, 6> jointCmd;

    // TODO: rename the following two variables:
    std::mutex m_read;  // guards jointFeedback
    std::mutex m_write;  // guards jointCmd

    uint16_t gripper_position = 0;
};

/**
 * @brief Structure representing the configuration of a joint motor.
 *
 * This structure is used to store the configuration of a joint motors in
 * arm_config file.
 */
struct jointConfig_t {
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

	uint8_t  invertDirection;
	uint8_t  requirePositioning;
	uint8_t  positioningOrder;
	float    positioningVelocity;
	uint16_t positioningTimeout;
	uint8_t  differential;
};

/**
 * @brief Structure representing the configuration of the arm.
 *
 * This structure is used to store the configuration of the arm (every joint) in
 * arm_config file.
 */
struct armConfig_t {
	uint8_t jointNumber;

	jointConfig_t joint[16];
	uint16_t      jointCommandRefreshTime_ms;
	uint16_t      jointCommunicationTimeout;
	uint16_t      canRoverStatusSendPeriod_ms;
};

enum ControlType { position, posvel };

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
