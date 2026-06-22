#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP

#include "can_types.hpp"

#include <mutex>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace CAN_driver {
#if defined(__clang__)
#define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define THREAD_ANNOTATION_ATTRIBUTE__(x)
#endif

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))
#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)
#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))
#define REQUIRES(...) THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))
#define ACQUIRE(...) THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))
#define RELEASE(...) THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))
#define TRY_ACQUIRE(...) THREAD_ANNOTATION_ATTRIBUTE__(try_acquire_capability(__VA_ARGS__))

class CAPABILITY("mutex") AnnotatedMutex
{
public:
  void lock() ACQUIRE() { mu_.lock(); }
  void unlock() RELEASE() { mu_.unlock(); }
  bool try_lock() TRY_ACQUIRE(true) { return mu_.try_lock(); }

private:
  std::mutex mu_;
};

class SCOPED_CAPABILITY MutexLock
{
public:
  explicit MutexLock(AnnotatedMutex& mu) ACQUIRE(mu) : mu_(mu) { mu_.lock(); }
  ~MutexLock() RELEASE() { mu_.unlock(); }

private:
  AnnotatedMutex& mu_;
};

extern "C" {
typedef struct DriverVars_t {
	int                 sock = 0;
	struct sockaddr_can addr = {};
	struct ifreq        ifr  = {};
	AnnotatedMutex          m_read;
	AnnotatedMutex          m_write;
	std::thread         reader;
	bool                should_run = true;
	~DriverVars_t() {
		this->should_run = false;
		this->reader.join();
		::close(this->sock);
	}
} DriverVars_t;
}
extern DriverVars_t arm_driver;

extern "C" {
int init(DriverVars_t *, const char *can_interface);
int write_gripper_position(DriverVars_t *driver_vars, uint16_t position);
int write_fastclick(DriverVars_t *driver_vars, uint8_t position);
}
int startArmRead();
int armRead();

int arm_write(ControlType controlType);
int write_control_type(ControlType controlType) REQUIRES(arm_driver.m_write);
int write_joint_setpoint(uint8_t joint_id) REQUIRES(arm_driver.m_write);
int write_joint_posvel(uint8_t joint_id) REQUIRES(arm_driver.m_write);
int write_data(
    DriverVars_t *driver_vars, uint16_t can_id, uint8_t *data, uint8_t len
);
int handle_frame(
    canfd_frame                                         frame,
    const std::unordered_map<uint8_t, canCmdHandler_t> *handles
)
    REQUIRES(arm_driver.m_read);
int close(DriverVars_t *driver_vars);
} // namespace CAN_driver

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP
