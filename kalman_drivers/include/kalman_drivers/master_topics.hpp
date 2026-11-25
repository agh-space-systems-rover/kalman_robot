// Names of the master_com topics, the drivers' contract with kalman_tunnel.
// C++ counterpart of kalman_drivers/master_topics.py.
#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace kalman_drivers::master_topics {

// Frames published here are sent to the Master.
inline constexpr const char *ROS_TO_MASTER = "master_com/ros_to_master";

// Topic of frames with ID cmd received from the Master, e.g. ".../x5a".
inline std::string master_to_ros(uint8_t cmd) {
	std::ostringstream name;
	name << "master_com/master_to_ros/x" << std::hex << static_cast<int>(cmd);
	return name.str();
}

} // namespace kalman_drivers::master_topics
