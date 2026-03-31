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

    /**
     * @brief Maximum number of joints that can be configured.
     *
     * Used for static sizing of std::array containers.
     */
    constexpr std::size_t K_MAX_JOINTS = 16;

/* ------------------------------------------------------------------ */
/*  CAN message handler structure                                    */
/* ------------------------------------------------------------------ */
struct canCmdHandler_t {
	uint16_t can_id{}; ///< CAN identifier
	uint8_t  len{};    ///< Length of payload
	int (*func)(
	    uint32_t identifier, uint8_t *data, uint8_t len
	); ///< Handler function
};

/* ------------------------------------------------------------------ */
/*  Joint status / setpoint structures                                 */
/* ------------------------------------------------------------------ */
struct jointMoveStatus_t {
	float velocity_deg_s{}; ///< Joint velocity (deg/s)
	float position_deg{};   ///< Joint position (deg)
};

struct jointMoveSetpoint_t {
	float torque_Nm{};           ///< Desired torque (Nm)
	float velocity_deg_s{};      ///< Setpoint velocity (deg/s)
	float position_deg{};        ///< Setpoint position (deg)
	float acceleration_deg_ss{}; ///< Setpoint acceleration (deg/s²)
};

struct jointStatus_t {
	jointMotorStatus_t     status{};
	jointMotorFastStatus_t fastStatus{};
	jointCmdSetpoint_t     setpoint{};
	jointCmdVelocity_t     velSetpoint{};
	jointMoveStatus_t      moveStatus{};
	jointMoveSetpoint_t    moveSetpoint{};
	jointMoveSetpoint_t    moveSetpointDiff{};
};

/* ------------------------------------------------------------------ */
/*  Joint configuration structures                                    */
/* ------------------------------------------------------------------ */
struct jointConfig_t {
	float maxVelocity_deg_s{};
	float maxAcceleration_deg_ss{};
	float maxTorque_Nm{};

	float minPosition_deg{};
	float maxPosition_deg{};

	float positionAfterPositioning{};
	float idleTorque_Nm{};
	float defVelocity_deg_s{};
	float defAcceleration_deg_ss{};
	float defAcceleration_pos_deg_ss{};

	float    gearRatio{};
	uint8_t  invertDirection{};
	uint8_t  requirePositioning{};
	uint8_t  positioningOrder{};
	float    positioningVelocity{};
	uint16_t positioningTimeout{};
	uint8_t  differential{};
};

struct armConfig_t {
	uint8_t                                 jointNumber{};
	std::array<jointConfig_t, K_MAX_JOINTS> joint{};
	uint16_t                                jointCommandRefreshTime_ms{};
	uint16_t                                jointCommunicationTimeout{};
	uint16_t                                canRoverStatusSendPeriod_ms{};
};

/* ------------------------------------------------------------------ */
/*  Control type – scoped enum                                         */
/* ------------------------------------------------------------------ */
enum class ControlType : uint8_t { position = 0, posvel = 1 };

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
