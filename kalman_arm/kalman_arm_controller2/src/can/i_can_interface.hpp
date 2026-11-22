#pragma once

#include <functional>
#include <vector>
#include <string>

#include "can/can_frame.hpp"

namespace kalman_arm::can {

class ICanInterface {
public:
  using RxCallback = std::function<void(const CanFrame&)>;

  virtual ~ICanInterface() = default;

  virtual bool open(const std::string& interface_name,
                    const std::vector<uint32_t>& rx_filter_ids,
                    bool enable_fd = true) = 0;
  virtual void close() = 0;
  virtual bool send(const CanFrame& frame) = 0;
  virtual void startReceive(RxCallback callback) = 0;
  virtual void stopReceive() = 0;
};

}  // namespace kalman_arm::can