#include "kalman_arm_controller/can_libs/can_messages.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
constexpr std::size_t kJointCount = 6;
constexpr auto kStatusPeriod = std::chrono::milliseconds{20};
constexpr auto kStatePrintPeriod = std::chrono::seconds{1};
constexpr uint32_t kCanIdMask = CAN_EFF_MASK;

volatile std::sig_atomic_t keep_running = 1;

void stop(int) { keep_running = 0; }

struct Joint {
  int32_t position = 0;
  int32_t target_position = 0;
  int16_t velocity = 0;  // 0.1 RPM, matches jointMotorFastStatus_t.
};

class ArmSimulator {
public:
  explicit ArmSimulator(std::string_view interface) : socket_(open_socket(interface)) {}

  ~ArmSimulator() {
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  ArmSimulator(const ArmSimulator &) = delete;
  ArmSimulator &operator=(const ArmSimulator &) = delete;

  int run() {
    std::cerr << "Arm CAN simulator running on " << interface_name_
              << "; control-C stops it.\n";

    auto previous_update = std::chrono::steady_clock::now();
    auto next_status = previous_update;
    while (keep_running) {
      const auto now = std::chrono::steady_clock::now();
      update_joints(std::chrono::duration<double>(now - previous_update).count());
      previous_update = now;

      if (now >= next_status) {
        publish_status();
        next_status = now + kStatusPeriod;
      }
      if (state_changed_since_print_ &&
          now - last_state_print_ >= kStatePrintPeriod) {
        print_joint_states();
        state_changed_since_print_ = false;
        last_state_print_ = now;
      }

      pollfd descriptor{socket_, POLLIN, 0};
      const int result = ::poll(&descriptor, 1, 2);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        std::cerr << "poll failed: " << std::strerror(errno) << '\n';
        return 1;
      }
      if (result > 0 && (descriptor.revents & POLLIN)) {
        receive_command();
      }
    }
    return 0;
  }

private:
  // Raw motor-position limits derived from arm_config.cpp: degree * 100 / gear ratio.
  static constexpr std::array<int32_t, kJointCount> kMinimumPosition{
      -1600000, -1920000, -2880000, -3600000, -763157, -27473684};
  static constexpr std::array<int32_t, kJointCount> kMaximumPosition{
      1600000, 1920000, 2880000, 3600000, 763157, 27473684};
  // Maximum raw position change per second. Limits feedback to configured motor speed.
  static constexpr std::array<double, kJointCount> kMaximumPositionRate{
      360000.0, 720000.0, 720000.0, 450000.0, 2747368.0, 2747368.0};
  static constexpr std::array<double, kJointCount> kGearRatio{
      0.0125, 0.00625, 0.00625, 0.01, 38.0 / 58.0 / 50.0, 38.0 / 58.0 / 50.0};
  static constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;

  std::string interface_name_;
  int socket_ = -1;
  std::array<Joint, kJointCount> joints_{};
  uint8_t control_mode_ = CONTROL_MODE_LEGACY;
  uint16_t gripper_position_ = 0;
  uint8_t fastclick_position_ = 0;
  bool state_changed_since_print_ = false;
  std::chrono::steady_clock::time_point last_state_print_{};

  int open_socket(std::string_view interface) {
    interface_name_ = interface;
    const int socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket < 0) {
      throw std::runtime_error(std::string{"socket failed: "} + std::strerror(errno));
    }

    const int enable_fd_frames = 1;
    if (::setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd_frames,
                     sizeof(enable_fd_frames)) < 0) {
      const std::string error = std::strerror(errno);
      ::close(socket);
      throw std::runtime_error("CAN FD enable failed: " + error);
    }

    const int receive_own_messages = 0;
    if (::setsockopt(socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                     &receive_own_messages, sizeof(receive_own_messages)) < 0) {
      const std::string error = std::strerror(errno);
      ::close(socket);
      throw std::runtime_error("CAN own-message configuration failed: " + error);
    }

    ifreq request{};
    if (interface.size() >= IFNAMSIZ) {
      ::close(socket);
      throw std::runtime_error("CAN interface name is too long");
    }
    std::memcpy(request.ifr_name, interface.data(), interface.size());
    if (::ioctl(socket, SIOCGIFINDEX, &request) < 0) {
      const std::string error = std::strerror(errno);
      ::close(socket);
      throw std::runtime_error("CAN interface lookup failed: " + error);
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      const std::string error = std::strerror(errno);
      ::close(socket);
      throw std::runtime_error("CAN bind failed: " + error);
    }
    return socket;
  }

  void receive_command() {
    canfd_frame frame{};
    const ssize_t bytes = ::read(socket_, &frame, sizeof(frame));
    if (bytes < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        std::cerr << "CAN read failed: " << std::strerror(errno) << '\n';
      }
      return;
    }
    if (bytes != CAN_MTU && bytes != CANFD_MTU) {
      reject("unexpected SocketCAN frame size");
      return;
    }
    if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
      reject("extended, remote, or error frame is not a command");
      return;
    }

    const uint32_t identifier = frame.can_id & kCanIdMask;
    const uint8_t joint = identifier >> 7;
    const uint8_t command = identifier & 0x7f;
    if (joint >= 1 && joint <= kJointCount) {
      if (command == CMD_JOINT_STATUS || command == CMD_JOINT_FAST_STATUS) {
        return;
      }
      handle_joint_command(joint - 1, command, frame);
      return;
    }

    switch (identifier) {
      case CMD_CONTROL_TYPE:
        handle_control_mode(frame);
        break;
      case CMD_SET_GRIPPER:
        handle_gripper(frame);
        break;
      case CMD_SET_FASTCLICK:
        handle_fastclick(frame);
        break;
      default:
        reject("unknown command CAN ID " + std::to_string(identifier));
    }
  }

  void handle_joint_command(std::size_t joint, uint8_t command,
                            const canfd_frame &frame) {
    if (command == CMD_SETPOINT) {
      if (!expect_length(frame, LEN_CMD_SETPOINT, "setpoint")) {
        return;
      }
      if (control_mode_ != CONTROL_MODE_POSITION) {
        reject("position setpoint received outside position mode");
        return;
      }
      int32_t target = 0;
      std::memcpy(&target, frame.data, sizeof(target));
      if (target < kMinimumPosition[joint] || target > kMaximumPosition[joint]) {
        reject("joint " + std::to_string(joint + 1) + " position outside configured limit");
        return;
      }
      joints_[joint].target_position = target;
      return;
    }

    if (command == CMD_VELOCITY) {
      if (!expect_length(frame, LEN_CMD_VELOCITY, "velocity")) {
        return;
      }
      if (control_mode_ != CONTROL_MODE_LEGACY) {
        reject("velocity command received outside legacy mode");
        return;
      }
      int16_t velocity = 0;
      std::memcpy(&velocity, frame.data, sizeof(velocity));
      if (joints_[joint].velocity != velocity) {
        joints_[joint].velocity = velocity;
        state_changed_since_print_ = true;
      }
      return;
    }

    reject("unknown joint command " + std::to_string(command));
  }

  void handle_control_mode(const canfd_frame &frame) {
    if (!expect_length(frame, LEN_CMD_CONTROL_TYPE, "control mode")) {
      return;
    }
    const uint8_t mode = frame.data[0];
    if (mode != CONTROL_MODE_POSITION && mode != CONTROL_MODE_LEGACY) {
      reject("unsupported control mode " + std::to_string(mode));
      return;
    }
    control_mode_ = mode;
  }

  void handle_gripper(const canfd_frame &frame) {
    if (!expect_length(frame, LEN_CMD_SET_GRIPPER, "gripper")) {
      return;
    }
    std::memcpy(&gripper_position_, frame.data, sizeof(gripper_position_));
  }

  void handle_fastclick(const canfd_frame &frame) {
    if (!expect_length(frame, LEN_CMD_SET_FASTCLICK, "fastclick")) {
      return;
    }
    fastclick_position_ = frame.data[0];
  }

  bool expect_length(const canfd_frame &frame, uint8_t expected,
                     std::string_view name) {
    if (frame.len == expected) {
      return true;
    }
    reject(std::string{name} + " frame has invalid length " + std::to_string(frame.len));
    return false;
  }

  void update_joints(double elapsed_seconds) {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      Joint &joint = joints_[index];
      const Joint previous = joint;
      if (control_mode_ == CONTROL_MODE_POSITION) {
        const double difference = static_cast<double>(joint.target_position) - joint.position;
        const double change = std::clamp(
            difference, -kMaximumPositionRate[index] * elapsed_seconds,
            kMaximumPositionRate[index] * elapsed_seconds);
        joint.position += static_cast<int32_t>(std::lround(change));
        joint.velocity = static_cast<int16_t>(std::clamp(
            std::lround(change / elapsed_seconds / 60.0),
            static_cast<long>(std::numeric_limits<int16_t>::min()),
            static_cast<long>(std::numeric_limits<int16_t>::max())));
      } else {
        const double change = joint.velocity * 60.0 * elapsed_seconds;
        const double position = std::clamp(
            static_cast<double>(joint.position) + change,
            static_cast<double>(kMinimumPosition[index]),
            static_cast<double>(kMaximumPosition[index]));
        joint.position = static_cast<int32_t>(std::lround(position));
      }
      if (joint.position != previous.position || joint.velocity != previous.velocity) {
        state_changed_since_print_ = true;
      }
    }
  }

  std::array<double, kJointCount> joint_angles_radians() const {
    std::array<double, kJointCount> motor_degrees{};
    for (std::size_t index = 0; index < kJointCount; ++index) {
      motor_degrees[index] = 0.01 * kGearRatio[index] * joints_[index].position;
    }

    // Mirrors CAN_vars::update_joint_status() and ArmSystem::read_joint_states().
    return {
        -motor_degrees[0] * kRadiansPerDegree,
        motor_degrees[1] * kRadiansPerDegree,
        -motor_degrees[2] * kRadiansPerDegree,
        motor_degrees[3] * kRadiansPerDegree,
        -(motor_degrees[4] - motor_degrees[5]) / 2.0 * kRadiansPerDegree,
        (motor_degrees[4] + motor_degrees[5]) / 2.0 * kRadiansPerDegree,
    };
  }

  void print_joint_states() const {
    const auto angles = joint_angles_radians();
    std::cerr << std::fixed << std::setprecision(3) << "Joint angles (rad):";
    for (std::size_t index = 0; index < kJointCount; ++index) {
      std::cerr << " j" << index + 1 << '=' << angles[index];
    }
    std::cerr << '\n';
  }

  void publish_status() {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      jointMotorFastStatus_t status{.velocity=joints_[index].velocity, .position=joints_[index].position};
      canfd_frame frame{};
      frame.can_id = static_cast<canid_t>(((index + 1) << 7) | CMD_JOINT_FAST_STATUS);
      frame.len = LEN_JOINT_FAST_STATUS;
      std::memcpy(frame.data, &status, sizeof(status));
      if (::write(socket_, &frame, sizeof(frame)) != CANFD_MTU) {
        std::cerr << "CAN status write failed: " << std::strerror(errno) << '\n';
      }
    }
  }

  void reject(const std::string &reason) const {
    std::cerr << "Rejected CAN command: " << reason << '\n';
  }
};
}  // namespace

int main(int argc, char **argv) {
  if (argc > 2) {
    std::cerr << "Usage: arm_can_simulator [can-interface]\n";
    return 2;
  }

  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try {
    ArmSimulator simulator{argc == 2 ? argv[1] : "can0"};
    return simulator.run();
  } catch (const std::exception &error) {
    std::cerr << "Failed to start arm CAN simulator: " << error.what() << '\n';
    return 1;
  }
}
