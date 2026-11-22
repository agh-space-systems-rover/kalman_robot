#include "can/socket_can_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

namespace kalman_arm::can {

SocketCanInterface::~SocketCanInterface() {
  stopReceive();
  close();
}

bool SocketCanInterface::open(const std::string& interface_name,
                              const std::vector<uint32_t>& rx_filter_ids,
                              bool enable_fd) {
  stopReceive();
  close();

  sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) return false;

  if (enable_fd) {
    int enable = 1;
    if (::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) == 0) {
      fd_enabled_ = true;
    }
  }

  if (!rx_filter_ids.empty()) {
    std::vector<struct can_filter> filters;
    filters.reserve(rx_filter_ids.size());
    for (uint32_t id : rx_filter_ids) {
      struct can_filter f{};
      f.can_id   = id;
      f.can_mask = (id > CAN_SFF_MASK) ? CAN_EFF_MASK : CAN_SFF_MASK;
      filters.push_back(f);
    }
    ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER,
                 filters.data(), filters.size() * sizeof(struct can_filter));
  }

  struct timeval tv{0, 100000};
  ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  return true;
}

void SocketCanInterface::close() {
  if (sock_ >= 0) {
    ::close(sock_);
    fd_enabled_ = false;
    sock_ = -1;
  }
}

bool SocketCanInterface::send(const CanFrame& frame) {
  if (sock_ < 0) return false;

  if (frame.is_fd ) {
    if (!fd_enabled_) return false;
    struct canfd_frame cf{};
    cf.can_id = frame.id | (frame.is_extended ? CAN_EFF_FLAG : 0);
    cf.len    = std::min<uint8_t>(frame.len, CANFD_MAX_DLEN);
    std::memcpy(cf.data, frame.data.data(), cf.len);
    return ::write(sock_, &cf, CANFD_MTU) == CANFD_MTU;
  }

  struct can_frame cf{};
  cf.can_id  = frame.id | (frame.is_extended ? CAN_EFF_FLAG : 0);
  cf.can_dlc = std::min<uint8_t>(frame.len, CAN_MAX_DLEN);
  std::memcpy(cf.data, frame.data.data(), cf.can_dlc);
  return ::write(sock_, &cf, CAN_MTU) == CAN_MTU;
}

void SocketCanInterface::startReceive(RxCallback callback) {
  if (sock_ < 0) return;
  stopReceive();
  rx_callback_ = std::move(callback);
  running_ = true;
  rx_thread_ = std::thread(&SocketCanInterface::rxLoop, this);
}

void SocketCanInterface::stopReceive() {
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void SocketCanInterface::rxLoop() {
  struct canfd_frame cf{};
  while (running_) {
    ssize_t nbytes = ::read(sock_, &cf, sizeof(cf));
    if (nbytes != CAN_MTU && nbytes != CANFD_MTU) {
      continue;
    }

    CanFrame out{};
    out.is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    out.id          = cf.can_id & (out.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out.is_fd       = (nbytes == CANFD_MTU);
    out.len         = std::min<uint8_t>(cf.len, out.is_fd ? CANFD_MAX_DLEN : CAN_MAX_DLEN);
    std::memcpy(out.data.data(), cf.data, out.len);

    if (rx_callback_) {
      rx_callback_(out);
    }
  }
}

}  // namespace kalman_arm::can
