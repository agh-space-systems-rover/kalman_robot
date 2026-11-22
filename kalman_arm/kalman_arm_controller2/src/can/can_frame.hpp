#pragma once

#include <array>
#include <cstdint>

namespace kalman_arm::can {

/**
 * @brief Represents a raw CAN 2.0 or CAN FD frame independent of Linux SocketCAN types.
 */
struct CanFrame {
  uint32_t id{0};                  ///< CAN arbitration ID (11-bit standard or 29-bit extended).
  uint8_t len{0};                  ///< Payload length in bytes (0..8 for CAN 2.0, 0..64 for CAN FD).
  bool is_fd{false};               ///< True if the frame uses CAN FD format; false for classic CAN 2.0.
  bool is_extended{false};         ///< True for 29-bit extended ID (EFF); false for 11-bit standard ID (SFF).
  std::array<uint8_t, 64> data{};  ///< Raw payload bytes (valid indices: 0 to len - 1).
};


}