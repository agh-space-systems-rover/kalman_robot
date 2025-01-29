#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP

#include "can_types.hpp"

#include <mutex>
#include <stdlib.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace CAN_driver
{
extern "C" {
typedef struct DriverVars_t
{
  int sock = 0;
  struct sockaddr_can addr = {};
  struct ifreq ifr = {};
  std::mutex m_read;
  std::mutex m_write;
  std::thread reader;
  bool should_run = true;
  ~DriverVars_t()
  {
    this->should_run = false;
    this->reader.join();
    ::close(this->sock);
  }
} DriverVars_t;
}
extern DriverVars_t arm_driver;

extern "C" {
int init(DriverVars_t*, const char* can_interface);
int write_gripper_position(DriverVars_t* driver_vars, uint16_t position);
int write_fastclick(DriverVars_t* driver_vars, uint8_t position);
}
int startArmRead();
int armRead();

int arm_write(ControlType controlType);
int write_control_type(ControlType controlType);
int write_joint_setpoint(uint8_t joint_id);
int write_joint_posvel(uint8_t joint_id);
int write_data(DriverVars_t* driver_vars, uint16_t can_id, uint8_t* data, uint8_t len);
int handle_frame(canfd_frame frame, const std::unordered_map<uint8_t, canCmdHandler_t>* handles);
int close(DriverVars_t* driver_vars);
}  // namespace CAN_driver

#endif  // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP