#include "kalman_arm_controller/can_libs/can_driver.hpp"
#include <cerrno>
#include <cstring>
#include <mutex>
#include <poll.h>
#include <vector>

#define BUFFER_SIZE 1024
#define TIMEOUT_MS 1  // 5 seconds

CAN_driver::DriverVars_t CAN_driver::arm_driver = {};
CAN_driver::DriverVars_t* CAN_driver::extra_driver = nullptr;
std::unordered_map<uint8_t, canCmdHandler_t>* CAN_driver::extra_handlers = nullptr;

/**
 * @brief Initialize the CAN driver.
 *
 * Initializes the CAN driver and binds the socket to the can0 interface
 *
 * @return int 0 on success, 1 on failure
 */
extern "C" {
int CAN_driver::init(DriverVars_t* driver_vars, const char* can_interface)
{
  printf("In CAN_driver::init\r\n");

  // Load default configuration
  arm_config::load_default_config();

  // Get socket connection
  if ((driver_vars->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Socket");
    return 1;
  }

  // Enable FD frames
  int enable_fd_frames = 1;
  setsockopt(driver_vars->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd_frames, sizeof(enable_fd_frames));

  // Set up the can interface
  strcpy(driver_vars->ifr.ifr_name, can_interface);
  ioctl(driver_vars->sock, SIOCGIFINDEX, &driver_vars->ifr);

  driver_vars->addr.can_family = AF_CAN;
  driver_vars->addr.can_ifindex = driver_vars->ifr.ifr_ifindex;

  // Bind the socket
  if (bind(driver_vars->sock, (struct sockaddr*)&driver_vars->addr, sizeof(driver_vars->addr)) < 0)
  {
    perror("Bind");
    return 1;
  }

  // Set timeout for reading
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1;
  setsockopt(driver_vars->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  printf("Finished CAN init! \r\n");
  return 0;
}
}
int CAN_driver::startArmRead()
{
  arm_driver.reader = std::thread(CAN_driver::armRead);
  return 0;
}

extern "C" {
int CAN_driver::startExtraRead(DriverVars_t* driver_vars, std::unordered_map<uint8_t, canCmdHandler_t>* extra_handles)
{
  driver_vars->reader = std::thread(CAN_driver::extraRead);
  CAN_driver::extra_driver = driver_vars;
  CAN_driver::extra_handlers = extra_handles;
  return 0;
}
}

int CAN_driver::extraRead()
{
  char buffer[BUFFER_SIZE];
  while (extra_driver->should_run)
  {
    ssize_t num_bytes = recv(extra_driver->sock, buffer, BUFFER_SIZE, MSG_DONTWAIT);

    if (num_bytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // there was nothing to read (recv MSG_DONTWAIT flag docs)
        std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
        continue;
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger("my_logger"), "CAN_driver::masterRead: recv failed due to: %s\r\n",
                     std::strerror(errno));
        exit(EXIT_FAILURE);
      }
    }

    buffer[num_bytes] = '\0';  // Null-terminate the received data
    struct canfd_frame frame;

    frame = *((struct canfd_frame*)buffer);

    // We don't need to lock the recv invocation
    std::lock_guard<std::mutex> lock(extra_driver->m_read);  // Yay for RAII
    handle_frame(frame, CAN_driver::extra_handlers);
  }
  return 0;
}

int CAN_driver::armRead()
{
  char buffer[BUFFER_SIZE];
  while (arm_driver.should_run)
  {
    ssize_t num_bytes = recv(arm_driver.sock, buffer, BUFFER_SIZE, MSG_DONTWAIT);

    if (num_bytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // there was nothing to read (recv MSG_DONTWAIT flag docs)
        std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
        continue;
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger("my_logger"), "CAN_driver::armRead: recv failed due to: %s\r\n",
                     std::strerror(errno));
        exit(EXIT_FAILURE);
      }
    }

    buffer[num_bytes] = '\0';  // Null-terminate the received data
    struct canfd_frame frame;

    frame = *((struct canfd_frame*)buffer);

    // We don't need to lock the recv invocation
    std::lock_guard<std::mutex> lock(arm_driver.m_read);  // Yay for RAII
    handle_frame(frame, &CAN_handlers::HANDLES);

    CAN_vars::update_joint_status();
  }
  return 0;
}

// int CAN_driver::read()
// {
//   char buffer[BUFFER_SIZE];
//   while (CAN_driver::should_run)
//   {
//     ssize_t num_bytes = recv(sock, buffer, BUFFER_SIZE, MSG_DONTWAIT);

//     if (num_bytes < 0)
//     {
//       if (errno == EAGAIN || errno == EWOULDBLOCK)
//       {
//         // there was nothing to read (recv MSG_DONTWAIT flag docs)
//         std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
//         continue;
//       }
//       else
//       {
//         RCLCPP_FATAL(rclcpp::get_logger("my_logger"), "CAN_driver::read: recv
//         failed due to: %s\r\n",
//                      std::strerror(errno));
//         exit(EXIT_FAILURE);
//       }
//     }

//     buffer[num_bytes] = '\0';  // Null-terminate the received data
//     struct canfd_frame frame;

//     frame = *((struct canfd_frame*)buffer);

//     // We don't need to lock the recv invocation
//     std::lock_guard<std::mutex> lock(CAN_driver::m_read);  // Yay for RAII
//     handle_frame(frame);

//     CAN_vars::update_joint_status();
//   }
//   return 0;
// }

/**
 * @brief Handle a received frame.
 *
 * Decodes the frame and calls the appropriate handler function.
 *
 * @param frame The frame to handle
 * @return int 0 on success, 1 on failure
 */
int CAN_driver::handle_frame(canfd_frame frame, std::unordered_map<uint8_t, canCmdHandler_t>* handles)
{
  // Decode the frame
  uint8_t joint_id = frame.can_id >> 7;
  uint8_t command = frame.can_id - (joint_id << 7);
  try
  {
    // RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Handling frame with ID %03X
    // and length %d, command: %d, joint id: %d\r\n", frame.can_id, frame.len,
    // command, joint_id);
    if (handles->find(command) != handles->end())
      (*handles)[command].func(frame.can_id, frame.data, frame.len);
  }
  catch (const std::exception& e)
  {
    // RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Caught exception: %s\r\n",
    // e.what());
    return 1;
  }
  return 0;
}

int CAN_driver::arm_write(ControlType controlType)
{
  std::lock_guard<std::mutex> lock(CAN_driver::arm_driver.m_write);

  CAN_vars::update_joint_setpoint();
  write_control_type(controlType);
  std::this_thread::sleep_for(std::chrono::microseconds(1000));

  // Write data from global joints
  for (int i = 0; i < 6; i++)
  {
    switch (controlType)
    {
      case ControlType::position:
        // printf("position\r\n");
        write_joint_setpoint(i);
        break;

      case ControlType::posvel:
        // printf("posvel\r\n");
        write_joint_posvel(i);
        break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  return 0;
}

int CAN_driver::write_control_type(ControlType controlType)
{
  jointCmdControlType_t data;
  switch (controlType)
  {
    case ControlType::position:
      data.controlMode = controlMode_t::CONTROL_MODE_POSITION;
      break;

    case ControlType::posvel:
      data.controlMode = controlMode_t::CONTROL_MODE_LEGACY;
      break;
  }

  uint16_t can_id = CMD_CONTROL_TYPE;
  return write_data(&arm_driver, can_id, (uint8_t*)&data, LEN_CMD_CONTROL_TYPE);
}

int CAN_driver::write_joint_setpoint(uint8_t joint_id)
{
  // Write data from a single joint
  joint_id += 1;

  uint16_t can_id = (joint_id << 7) + CMD_SETPOINT;
  if (1 <= joint_id && joint_id <= 6)
  {
    return write_data(&arm_driver, can_id, (uint8_t*)&CAN_vars::joints[joint_id - 1].setpoint,
                      sizeof(jointCmdSetpoint_t));
  }
  return 1;
}

int CAN_driver::write_joint_posvel(uint8_t joint_id)
{
  joint_id += 1;

  uint16_t can_id = (joint_id << 7) + CMD_VELOCITY;
  if (1 <= joint_id && joint_id <= 6)
  {
    return write_data(&arm_driver, can_id, (uint8_t*)&CAN_vars::joints[joint_id - 1].velSetpoint,
                      sizeof(jointCmdVelocity_t));
  }
  return 1;
}

extern "C" {
int CAN_driver::write_gripper_position(DriverVars_t* driver_vars, uint16_t position)
{
  cmdSetGripper_t data;
  data.gripperPosition = position;
  uint16_t can_id = CMD_SET_GRIPPER;
  return write_data(driver_vars, can_id, (uint8_t*)&data, LEN_CMD_SET_GRIPPER);
}
}

int CAN_driver::write_data(DriverVars_t* driver_vars, uint16_t can_id, uint8_t* data, uint8_t len)
{
  struct canfd_frame frame;
  frame.can_id = can_id;
  frame.len = len;
  frame.flags = 0;
  memcpy(frame.data, data, len);
  if (::write(driver_vars->sock, &frame, sizeof(frame)) < 0)
  {
    perror("Write");
    return 1;
  }
  return 0;
}

int CAN_driver::close(DriverVars_t* driver_vars)
{
  driver_vars->should_run = false;
  driver_vars->reader.join();
  return (::close(driver_vars->sock) < 0);
}