#include "gripper/gripper_can_protocol.hpp"

#include <algorithm>

namespace kalman_arm::gripper {

namespace {

inline void writeU16LE(uint8_t* dst, uint16_t val) {
  dst[0] = static_cast<uint8_t>(val & 0xFF);
  dst[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

inline void writeI16LE(uint8_t* dst, int16_t val) {
  writeU16LE(dst, static_cast<uint16_t>(val));
}

inline uint16_t readU16LE(const uint8_t* src) {
  return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8);
}

inline int16_t readI16LE(const uint8_t* src) {
  return static_cast<int16_t>(readU16LE(src));
}

}  // namespace

can::CanFrame GripperCanProtocol::encodeSetPosition(int16_t upper_deg, int16_t lower_deg) {
  can::CanFrame f{};
  f.id  = ID_SET_POS;
  f.len = 4;
  writeI16LE(&f.data[0], upper_deg);
  writeI16LE(&f.data[2], lower_deg);
  return f;
}

can::CanFrame GripperCanProtocol::encodeSoftReset() {
  can::CanFrame f{};
  f.id  = ID_SOFT_RESET;
  f.len = 0;
  return f;
}

can::CanFrame GripperCanProtocol::encodeAutoclickPos(uint8_t pos_deg) {
  can::CanFrame f{};
  f.id      = ID_AUTOCLICK_POS;
  f.len     = 1;
  f.data[0] = std::min<uint8_t>(pos_deg, MAX_AUTOCLICK_DEG);
  return f;
}

can::CanFrame GripperCanProtocol::encodeSetTorqueLimit(uint16_t upper_limit, uint16_t lower_limit) {
  can::CanFrame f{};
  f.id  = ID_SET_TORQUE_LIMIT;
  f.len = 4;
  writeU16LE(&f.data[0], std::min(upper_limit, MAX_TORQUE_LIMIT));
  writeU16LE(&f.data[2], std::min(lower_limit, MAX_TORQUE_LIMIT));
  return f;
}

can::CanFrame GripperCanProtocol::encodeCalibrate() {
  can::CanFrame f{};
  f.id  = ID_CALIBRATE;
  f.len = 0;
  return f;
}

std::optional<DualPositionDeg> GripperCanProtocol::decodePosition(const can::CanFrame& frame) {
  if (frame.id != ID_RETURN_POS || frame.len < 4) {
    return std::nullopt;
  }
  return DualPositionDeg{
    readI16LE(&frame.data[0]),
    readI16LE(&frame.data[2])
  };
}

std::optional<DualTempC> GripperCanProtocol::decodeTemperature(const can::CanFrame& frame) {
  if (frame.id != ID_RETURN_TEMP || frame.len < 2) {
    return std::nullopt;
  }
  return DualTempC{frame.data[0], frame.data[1]};
}

std::optional<DualTorque> GripperCanProtocol::decodeTorque(const can::CanFrame& frame) {
  if (frame.id != ID_RETURN_TORQUE || frame.len < 4) {
    return std::nullopt;
  }
  return DualTorque{
    readU16LE(&frame.data[0]),
    readU16LE(&frame.data[2])
  };
}

std::optional<CalibrationReport> GripperCanProtocol::decodeCalibrationStatus(
    const can::CanFrame& frame) {
  if (frame.id != ID_CALIB_STATUS || frame.len < 9 || frame.data[0] > 2) {
    return std::nullopt;
  }
  CalibrationReport r{};
  r.status        = static_cast<CalibrationStatus>(frame.data[0]);
  r.upper_min_deg = readI16LE(&frame.data[1]);
  r.upper_max_deg = readI16LE(&frame.data[3]);
  r.lower_min_deg = readI16LE(&frame.data[5]);
  r.lower_max_deg = readI16LE(&frame.data[7]);
  return r;
}

}  // namespace kalman_arm::gripper