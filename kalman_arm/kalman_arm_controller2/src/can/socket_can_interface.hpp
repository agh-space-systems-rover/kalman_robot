#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "can/i_can_interface.hpp"

namespace kalman_arm::can {

class SocketCanInterface : public ICanInterface {
public:
  SocketCanInterface() = default;
  ~SocketCanInterface() override;

  SocketCanInterface(const SocketCanInterface&) = delete;
  SocketCanInterface& operator=(const SocketCanInterface&) = delete;

  bool open(const std::string& interface_name,
            const std::vector<uint32_t>& rx_filter_ids,
            bool enable_fd = true) override;
  void close() override;
  bool send(const CanFrame& frame) override;
  void startReceive(RxCallback callback) override;
  void stopReceive() override;

private:
  void rxLoop();

  int sock_{-1};
  bool fd_enabled_{false};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  RxCallback rx_callback_;
};

}  // namespace kalman_arm::can