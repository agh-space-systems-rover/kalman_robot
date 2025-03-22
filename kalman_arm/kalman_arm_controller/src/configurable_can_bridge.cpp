#include "configurable_can_bridge.hpp"

#include <asm-generic/socket.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/serialization.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

namespace configurable_can_bridge
{

CANMapping::CANMapping(uint32_t can_id, std::string ros_topic, std::string ros_type, std::string conversion_type)
  : can_id(can_id)
  , ros_topic(ros_topic)
  , ros_type(ros_type)
  , conversion_type(conversion_type)
  , extended_id(can_id > 0x7FF)
{
}

CANBridge::CANBridge(const rclcpp::NodeOptions& options) : Node("configurable_can_bridge", options), can_socket_(-1)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Configurable CAN Bridge");

  // Load configuration
  if (!loadConfig())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load configuration");
    return;
  }

  // Setup message converters
  setupConverters();

  // Initialize CAN socket
  if (!initializeCANSocket())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket");
    return;
  }

  // Process mappings to create publishers and subscribers
  for (const auto& mapping : rx_mappings_)
  {
    processRxMapping(mapping);
  }

  for (const auto& mapping : tx_mappings_)
  {
    processTxMapping(mapping);
  }

  // Start CAN reception timer
  can_receive_timer_ = this->create_wall_timer(10ms, std::bind(&CANBridge::receiveCANMessages, this));

  RCLCPP_INFO(this->get_logger(), "CAN Bridge initialized successfully");
}

CANBridge::~CANBridge()
{
  if (can_socket_ >= 0)
  {
    close(can_socket_);
  }
}

bool CANBridge::loadConfig()
{
  // Declare and get parameters
  auto config_file = declare_parameter<std::string>("config_file", "");
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");

  if (config_file.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No configuration file specified");
    return false;
  }

  try
  {
    YAML::Node config = YAML::LoadFile(config_file);

    // Override can_interface if specified in config
    if (config["can_interface"])
    {
      can_interface_ = config["can_interface"].as<std::string>();
    }

    // Process RX mappings (CAN to ROS)
    if (config["rx_mappings"])
    {
      for (const auto& mapping : config["rx_mappings"])
      {
        uint32_t can_id = mapping["can_id"].as<uint32_t>();
        std::string ros_topic = mapping["ros_topic"].as<std::string>();
        std::string ros_type = mapping["ros_type"].as<std::string>();
        std::string conversion_type = "direct";

        if (mapping["conversion"] && mapping["conversion"]["type"])
        {
          conversion_type = mapping["conversion"]["type"].as<std::string>();
        }

        rx_mappings_.emplace_back(can_id, ros_topic, ros_type, conversion_type);
      }
    }

    // Process TX mappings (ROS to CAN)
    if (config["tx_mappings"])
    {
      for (const auto& mapping : config["tx_mappings"])
      {
        uint32_t can_id = mapping["can_id"].as<uint32_t>();
        std::string ros_topic = mapping["ros_topic"].as<std::string>();
        std::string ros_type = mapping["ros_type"].as<std::string>();
        std::string conversion_type = "direct";

        if (mapping["conversion"] && mapping["conversion"]["type"])
        {
          conversion_type = mapping["conversion"]["type"].as<std::string>();
        }

        tx_mappings_.emplace_back(can_id, ros_topic, ros_type, conversion_type);
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %ld RX mappings and %ld TX mappings", rx_mappings_.size(), tx_mappings_.size());

    return true;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error parsing config file: %s", e.what());
    return false;
  }
}

bool CANBridge::initializeCANSocket()
{
  // Open CAN socket
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "Error opening CAN socket: %s", strerror(errno));
    return false;
  }

  // Enable FD frames
  int enable_fd_frames = 1;
  setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd_frames, sizeof(enable_fd_frames));

  // Configure CAN interface
  struct ifreq ifr;
  strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Error getting interface index for %s: %s", can_interface_.c_str(), strerror(errno));
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Error binding CAN socket: %s", strerror(errno));
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // Set timeout for reading
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1;
  setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  RCLCPP_INFO(get_logger(), "CAN socket initialized successfully on %s", can_interface_.c_str());
  return true;
}

void CANBridge::processRxMapping(const CANMapping& mapping)
{
  auto publisher = createPublisher(mapping);
  if (publisher)
  {
    publishers_[mapping.can_id] = publisher;
    RCLCPP_INFO(get_logger(), "Created publisher for CAN ID 0x%X -> %s", mapping.can_id, mapping.ros_topic.c_str());
  }
}

void CANBridge::processTxMapping(const CANMapping& mapping)
{
  auto subscriber = createSubscriber(mapping);
  if (subscriber)
  {
    subscribers_.push_back(subscriber);
    RCLCPP_INFO(get_logger(), "Created subscriber for %s -> CAN ID 0x%X", mapping.ros_topic.c_str(), mapping.can_id);
  }
}

rclcpp::GenericPublisher::SharedPtr CANBridge::createPublisher(const CANMapping& mapping)
{
  try
  {
    return this->create_generic_publisher(mapping.ros_topic, mapping.ros_type, rclcpp::QoS(10));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create publisher for topic %s with type %s: %s", mapping.ros_topic.c_str(),
                 mapping.ros_type.c_str(), e.what());
    return nullptr;
  }
}

rclcpp::GenericSubscription::SharedPtr CANBridge::createSubscriber(const CANMapping& mapping)
{
  try
  {
    auto callback = [this, mapping](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      if (mapping.ros_type == "std_msgs/msg/Int32")
      {
        handleROStoCAN<std_msgs::msg::Int32>(msg, mapping);
      }
      else if (mapping.ros_type == "std_msgs/msg/Float32")
      {
        handleROStoCAN<std_msgs::msg::Float32>(msg, mapping);
      }
      else if (mapping.ros_type == "std_msgs/msg/Bool")
      {
        handleROStoCAN<std_msgs::msg::Bool>(msg, mapping);
      }
      else if (mapping.ros_type == "std_msgs/msg/UInt8MultiArray")
      {
        handleROStoCAN<std_msgs::msg::UInt8MultiArray>(msg, mapping);
      }
      else if (mapping.ros_type == "std_msgs/msg/UInt8")
      {
        handleROStoCAN<std_msgs::msg::UInt8>(msg, mapping);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Unsupported ROS type for CAN conversion: %s", mapping.ros_type.c_str());
      }
    };

    return this->create_generic_subscription(mapping.ros_topic, mapping.ros_type, rclcpp::QoS(10), callback);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create subscriber for topic %s with type %s: %s", mapping.ros_topic.c_str(),
                 mapping.ros_type.c_str(), e.what());
    return nullptr;
  }
}

void CANBridge::receiveCANMessages()
{
  if (can_socket_ < 0)
  {
    return;
  }

  struct canfd_frame frame;
  while (true)
  {
    ssize_t nbytes = read(can_socket_, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // No more data available
        break;
      }
      RCLCPP_ERROR(get_logger(), "Error reading from CAN socket: %s", strerror(errno));
      break;
    }

    // if (nbytes < sizeof(struct canfd_frame))
    // {
    //   RCLCPP_WARN(get_logger(), "Incomplete CAN frame received: %ld bytes, expecting %ld", nbytes,
    //               sizeof(struct canfd_frame));
    //   continue;
    // }

    handleCANtoROS(frame);
  }
}

bool CANBridge::sendCANMessage(const canfd_frame& frame)
{
  if (can_socket_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "CAN socket not initialized");
    return false;
  }

  ssize_t nbytes = write(can_socket_, &frame, sizeof(struct canfd_frame));
  if (nbytes != sizeof(struct canfd_frame))
  {
    RCLCPP_ERROR(get_logger(), "Error sending CAN frame: %s", strerror(errno));
    return false;
  }

  return true;
}

void CANBridge::handleCANtoROS(const canfd_frame& frame)
{
  auto it = publishers_.find(frame.can_id);
  if (it == publishers_.end())
  {
    // No mapping for this CAN ID
    return;
  }

  // Find the corresponding mapping
  const CANMapping* mapping = nullptr;
  for (const auto& m : rx_mappings_)
  {
    if (m.can_id == frame.can_id)
    {
      mapping = &m;
      break;
    }
  }

  //   if (!mapping)
  //   {
  //     RCLCPP_ERROR(get_logger(), "Inconsistent state: Publisher exists but
  //     mapping not found"); return;
  //   }
  if (mapping)
  {
    RCLCPP_INFO(get_logger(), "Received CAN message for %s", mapping->ros_topic.c_str());
  }

  // Find the appropriate converter for the message type
  auto converter_it = can_to_ros_converters_.find(mapping->ros_type);
  if (converter_it != can_to_ros_converters_.end())
  {
    converter_it->second(frame, it->second, mapping->conversion_type);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "No converter available for ROS type: %s", mapping->ros_type.c_str());
  }
}

template <typename T>
void CANBridge::handleROStoCAN(const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
                               const CANMapping& mapping)
{
  auto msg = std::make_shared<T>();
  rclcpp::Serialization<T> serialization;
  serialization.deserialize_message(serialized_msg.get(), msg.get());

  struct canfd_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = mapping.can_id;

  if (mapping.extended_id)
  {
    frame.can_id |= CAN_EFF_FLAG;
  }

  // Set DLC and data based on message type
  if constexpr (std::is_same_v<T, std_msgs::msg::Int32>)
  {
    frame.len = 4;
    int32_t value = msg->data;
    std::memcpy(frame.data, &value, 4);
  }
  else if constexpr (std::is_same_v<T, std_msgs::msg::Float32>)
  {
    frame.len = 4;
    float value = msg->data;
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    std::memcpy(frame.data, &raw, 4);
  }
  else if constexpr (std::is_same_v<T, std_msgs::msg::Bool>)
  {
    frame.len = 1;
    frame.data[0] = msg->data ? 1 : 0;
  }
  else if constexpr (std::is_same_v<T, std_msgs::msg::UInt8MultiArray>)
  {
    size_t length = std::min(msg->data.size(), static_cast<size_t>(8));
    frame.len = length;
    for (size_t i = 0; i < length; i++)
    {
      frame.data[i] = msg->data[i];
    }
  }
  else if constexpr (std::is_same_v<T, std_msgs::msg::UInt8>)
  {
    frame.len = 1;
    frame.data[0] = msg->data;
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Unsupported type for ROS to CAN conversion");
    return;
  }

  sendCANMessage(frame);
}

void CANBridge::setupConverters()
{
  // Register converters for different message types
  can_to_ros_converters_["std_msgs/msg/Int32"] =
      [](const canfd_frame& frame, rclcpp::GenericPublisher::SharedPtr publisher, std::string conversion_type) {
        auto msg = std::make_shared<std_msgs::msg::Int32>();
        if (frame.len >= 4)
        {
          if (conversion_type == "direct")
          {
            msg->data = static_cast<int32_t>(
                (static_cast<uint32_t>(frame.data[0]) << 0) | (static_cast<uint32_t>(frame.data[1]) << 8) |
                (static_cast<uint32_t>(frame.data[2]) << 16) | (static_cast<uint32_t>(frame.data[3]) << 24));
          }
          else
          {
            msg->data = 0;
          }
        }
        else
        {
          msg->data = 0;
        }

        auto serialized_msg = serialize_message(*msg);
        publisher->publish(*serialized_msg);
      };

  can_to_ros_converters_["std_msgs/msg/Float32"] =
      [](const canfd_frame& frame, rclcpp::GenericPublisher::SharedPtr publisher, std::string conversion_type) {
        auto msg = std::make_shared<std_msgs::msg::Float32>();
        if (frame.len >= 4)
        {
          if (conversion_type == "direct")
          {
            uint32_t raw = (static_cast<uint32_t>(frame.data[0]) << 0) | (static_cast<uint32_t>(frame.data[1]) << 8) |
                           (static_cast<uint32_t>(frame.data[2]) << 16) | (static_cast<uint32_t>(frame.data[3]) << 24);
            std::memcpy(&msg->data, &raw, 4);
          }
          else
          {
            msg->data = 0.0f;
          }
        }
        else
        {
          msg->data = 0.0f;
        }

        auto serialized_msg = serialize_message(*msg);
        publisher->publish(*serialized_msg);
      };

  can_to_ros_converters_["std_msgs/msg/Bool"] =
      [](const canfd_frame& frame, rclcpp::GenericPublisher::SharedPtr publisher, std::string conversion_type) {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        if (frame.len >= 1)
        {
          if (conversion_type == "direct")
          {
            msg->data = frame.data[0] != 0;
          }
          else
          {
            msg->data = false;
          }
        }
        else
        {
          msg->data = false;
        }

        auto serialized_msg = serialize_message(*msg);
        publisher->publish(*serialized_msg);
      };

  can_to_ros_converters_["std_msgs/msg/UInt8MultiArray"] =
      [](const canfd_frame& frame, rclcpp::GenericPublisher::SharedPtr publisher, std::string conversion_type) {
        auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();

        if (conversion_type == "direct")
        {
          msg->data.resize(frame.len);
          for (unsigned int i = 0; i < frame.len; i++)
          {
            msg->data[i] = frame.data[i];
          }
        }
        else
        {
          msg->data.resize(frame.len);
          for (unsigned int i = 0; i < frame.len; i++)
          {
            msg->data[i] = frame.data[i];
          }
        }

        auto serialized_msg = serialize_message(*msg);
        publisher->publish(*serialized_msg);
      };

  can_to_ros_converters_["std_msgs/msg/UInt8"] =
      [](const canfd_frame& frame, rclcpp::GenericPublisher::SharedPtr publisher, std::string conversion_type) {
        auto msg = std::make_shared<std_msgs::msg::UInt8>();
        if (frame.len >= 1)
        {
          if (conversion_type == "direct")
          {
            msg->data = frame.data[0];
          }
          else
          {
            msg->data = 0;
          }
        }
        else
        {
          msg->data = 0;
        }

        auto serialized_msg = serialize_message(*msg);
        publisher->publish(*serialized_msg);
      };
}

template <typename MessageT>
std::shared_ptr<rclcpp::SerializedMessage> serialize_message(const MessageT& message)
{
  // Create a serialized message object
  auto serialized_message = std::make_shared<rclcpp::SerializedMessage>();

  // Create a serialization object for the specific message type
  rclcpp::Serialization<MessageT> serialization;

  // Serialize the message
  serialization.serialize_message(&message, serialized_message.get());

  return serialized_message;
}

}  // namespace configurable_can_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(configurable_can_bridge::CANBridge)