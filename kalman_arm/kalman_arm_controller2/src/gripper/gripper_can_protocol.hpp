#pragma once

#include <cstdint>
#include <optional>

#include "can/can_frame.hpp"

namespace kalman_arm::gripper {

enum class CalibrationStatus : uint8_t {
  Ok         = 0,
  InProgress = 1,
  Error      = 2
};

struct DualPositionDeg {
  int16_t upper{0};
  int16_t lower{0};
};

struct DualTempC {
  uint8_t upper{0};
  uint8_t lower{0};
};

struct DualTorque {
  uint16_t upper{0};
  uint16_t lower{0};
};

struct CalibrationReport {
  CalibrationStatus status{CalibrationStatus::Ok};
  int16_t upper_min_deg{0};
  int16_t upper_max_deg{0};
  int16_t lower_min_deg{0};
  int16_t lower_max_deg{0};
};

class GripperCanProtocol {
public:
  // Master -> Gripper (RX na STM32)
  static constexpr uint32_t ID_SET_POS          = 0xE1;
  static constexpr uint32_t ID_SOFT_RESET       = 0xE2;
  static constexpr uint32_t ID_AUTOCLICK_POS    = 0xE3;
  static constexpr uint32_t ID_SET_TORQUE_LIMIT = 0xE7;
  static constexpr uint32_t ID_CALIBRATE        = 0xEC;

  // Gripper -> Master (TX z STM32)
  static constexpr uint32_t ID_RETURN_POS       = 0xE8;
  static constexpr uint32_t ID_RETURN_TEMP      = 0xE9;
  static constexpr uint32_t ID_CALIB_STATUS     = 0xED;
  static constexpr uint32_t ID_RETURN_TORQUE    = 0xEF;

  static constexpr uint16_t MAX_TORQUE_LIMIT = 1000;
  static constexpr uint8_t  MAX_AUTOCLICK_DEG = 180;

  // Encoders
  static can::CanFrame encodeSetPosition(int16_t upper_deg, int16_t lower_deg);
  static can::CanFrame encodeSoftReset();
  static can::CanFrame encodeAutoclickPos(uint8_t pos_deg);
  static can::CanFrame encodeSetTorqueLimit(uint16_t upper_limit, uint16_t lower_limit);
  static can::CanFrame encodeCalibrate();

  // Decoders
  static std::optional<DualPositionDeg>   decodePosition(const can::CanFrame& frame);
  static std::optional<DualTempC>         decodeTemperature(const can::CanFrame& frame);
  static std::optional<DualTorque>        decodeTorque(const can::CanFrame& frame);
  static std::optional<CalibrationReport> decodeCalibrationStatus(const can::CanFrame& frame);
};

}  // namespace kalman_arm::gripper