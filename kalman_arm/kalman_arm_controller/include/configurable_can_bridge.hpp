#ifndef CONFIGURABLE_CAN_BRIDGE__CAN_BRIDGE_HPP_
#define CONFIGURABLE_CAN_BRIDGE__CAN_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <yaml-cpp/yaml.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

namespace configurable_can_bridge
{

class CANMapping
{
public:
  CANMapping(uint32_t can_id, std::string ros_topic, std::string ros_type, float max_rate, std::string conversion_type);

  uint32_t can_id;
  std::string ros_topic;
  std::string ros_type;
  float max_rate;
  std::string conversion_type;

  bool extended_id;
  rclcpp::Time last_sent_stamp;
};

class CANBridge : public rclcpp::Node
{
public:
  explicit CANBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~CANBridge();

private:
  // CAN socket handling
  int can_socket_;
  std::string can_interface_;

  // Configuration
  std::vector<CANMapping> rx_mappings_;
  std::vector<CANMapping> tx_mappings_;

  // ROS publishers (for CAN to ROS)
  std::unordered_map<uint32_t, rclcpp::GenericPublisher::SharedPtr> publishers_;

  // ROS subscribers (for ROS to CAN)
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscribers_;

  // Message processing functions
  void processTxMapping(CANMapping& mapping);
  void processRxMapping(CANMapping& mapping);

  // CAN socket operations
  bool initializeCANSocket();
  void receiveCANMessages();
  bool sendCANMessage(const canfd_frame& frame);

  // Configuration loading
  bool loadConfig();

  // ROS timer for CAN message reception
  rclcpp::TimerBase::SharedPtr can_receive_timer_;

  // Message conversion methods
  void handleCANtoROS(const canfd_frame& frame);

  template <typename T>
  void handleROStoCAN(const std::shared_ptr<rclcpp::SerializedMessage> msg, CANMapping* mapping);

  // Helper methods for dynamic message type handling
  rclcpp::GenericPublisher::SharedPtr createPublisher(const CANMapping& mapping);
  rclcpp::GenericSubscription::SharedPtr createSubscriber(CANMapping* mapping);

  // Message conversion utilities
  std::unordered_map<std::string,
                     std::function<void(const canfd_frame&, rclcpp::GenericPublisher::SharedPtr, std::string)>>
      can_to_ros_converters_;

  template <typename T>
  void registerConverter(const std::string& name);

  void setupConverters();
};

template <typename MessageT>
std::shared_ptr<rclcpp::SerializedMessage> serialize_message(const MessageT& message);

}  // namespace configurable_can_bridge

#endif  // CONFIGURABLE_CAN_BRIDGE__CAN_BRIDGE_HPP_