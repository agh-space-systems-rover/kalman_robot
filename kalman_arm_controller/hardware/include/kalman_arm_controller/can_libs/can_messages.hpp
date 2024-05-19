#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_MESSAGES_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_MESSAGES_HPP
#include <stdint.h>

/*********RECEIVED MESSAGES***********/

/**
 * @brief Enumeration representing the positioning status.
 */
typedef enum {
  POSITIONING_NO,          ///< No positioning
  POSITIONING_IN_PROGRESS, ///< Positioning in progress
  POSITIONING_TIMEOUT,     ///< Positioning timeout
  POSITIONING_ERROR,       ///< Positioning error
  POSITIONING_SUCCESS,     ///< Positioning success
  POSITIONING_ENUM_SIZE    ///< Size of the positioning status enum
} positioningStatus_t;

typedef enum {
  CONTROL_MODE_SPEED = 0,
  CONTROL_MODE_POSITION = 1,
  CONTROL_MODE_LEGACY = 2,
} controlMode_t;

/**
 * @brief Structure representing the status of a joint motor received from the
 * CAN bus.
 *
 * This structure stores the status of the joint motor received from the CAN
 * bus.
 *
 * @param fault struct __attribute__((__packed__)) {
 *      uint8_t motorTemperatureFault : 1;
 *      uint8_t controllerTemperatureFault : 1;
 *      uint8_t encoderFault : 1;
 *      uint8_t timeout : 1;
 *      uint8_t positioningError : 1;
 *      uint8_t VinToLow : 1;
 *      uint8_t reserved : 2;
 * } bit;
 * @param fault.allFault uint8_t Fault status - byte 0
 * @param outputEnable uint8_t Output enable - byte 1.0
 * @param commandsBlocked uint8_t Commands blocked - byte 1.1
 * @param motorInitialized uint8_t Motor initialized - byte 1.2
 * @param positioningStatus positioningStatus_t Positioning status - byte 1.3-5
 * @param switchState uint8_t Switch state - byte 1.6-7
 * @param motorTemperature_1deg int8_t Motor temperature in degrees Celsius -
 * byte 2
 * @param controllerTemerature_1deg int8_t Controller temperature in degrees
 * Celsius - byte 3
 * @param torque int16_t Torque - bytes 4-5
 * @param velocity int16_t Velocity - bytes 6-7
 * @param position int32_t Position - bytes 8-11
 * @param inputVoltage_0V2 uint8_t Input voltage in 0.2V units - byte 12
 * @param reserved uint8_t[3] Reserved bytes - bytes 13-15
 */
typedef struct __attribute__((__packed__)) {
  union {
    struct __attribute__((__packed__)) {
      uint8_t motorTemperatureFault : 1;
      uint8_t controllerTemperatureFault : 1;
      uint8_t encoderFault : 1;
      uint8_t timeout : 1;
      uint8_t positioningError : 1;
      uint8_t VinToLow : 1;
      uint8_t reserved : 2;
    } bit;

    uint8_t allFault;
  } fault;

  uint8_t outputEnable : 1;
  uint8_t commandsBlocked : 1;
  uint8_t motorInitialized : 1;
  positioningStatus_t positioningStatus : 3;
  uint8_t switchState : 2;

  int8_t motorTemperature_1deg;
  int8_t controllerTemerature_1deg;

  int16_t torque;
  int16_t velocity; // RPM*10
  int32_t position; // pozycja 0-36000 (co 0.01 deg)

  uint8_t inputVoltage_0V2;

  uint8_t reserved[3];
} jointMotorStatus_t;
#define CMD_JOINT_STATUS 0x030
#define LEN_JOINT_STATUS 16

/**
 * @brief Structure representing the fast status (only vel and pos).
 *
 * @param velocity int16_t Velocity - bytes 0-1
 * @param position int32_t Position - bytes 2-5
 */
typedef struct __attribute__((__packed__)) {
  int16_t velocity; // RPM*10
  int32_t position; // pozycja 0-36000 (co 0.01 deg)
} jointMotorFastStatus_t;
#define CMD_JOINT_FAST_STATUS 0x036
#define LEN_JOINT_FAST_STATUS 6

/*********SENDING MESSAGES************/

/**
 * @brief Structure representing the setpoint to be sent to a joint motor
 * through the CAN bus.
 *
 * This structure is used to send the setpoint to the joint motor through the
 * CAN bus.
 *
 * @param position_0deg01 int32_t Position in 0.01 degrees units - bytes 0-3
 */
typedef struct __attribute__((__packed__)) {
  int32_t position_0deg01;
} jointCmdSetpoint_t;
#define CMD_SETPOINT 0x026
#define LEN_CMD_SETPOINT 4

/**
 * @brief Structure representing the velocity to be sent to a joint motor
 * through the CAN bus.
 *
 * This structure is used to send the velocity to the joint motor through the
 * CAN bus.
 *
 * @param velocity_0RPM_1 int16_t Velocity in 0.1 RPM units - bytes 0-1
 */
typedef struct __attribute__((__packed__)) {
  int16_t velocity_0RPM_1;
} jointCmdVelocity_t;
#define CMD_VELOCITY 0x025
#define LEN_CMD_VELOCITY 2

/**
 * @brief Structure representing the control mode to be sent to a joint motor
 * through the CAN bus.
 *
 * Depending on the control mode, joint motor will be controlled in different
 * ways by the microcontroller.
 *
 * @param controlMode controlMode_t Control mode - byte 0
 */
typedef struct __attribute__((__packed__)) {
  controlMode_t controlMode;
} jointCmdControlType_t;
#define CMD_CONTROL_TYPE 0x035
#define LEN_CMD_CONTROL_TYPE 1

typedef struct __attribute__((__packed__)) {
  uint16_t gripperPosition;
} cmdSetGripper_t;
#define CMD_SET_GRIPPER 0xE3
#define LEN_CMD_SET_GRIPPER 2

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_MESSAGES_HPP
