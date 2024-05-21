#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <rclcpp/qos_event.hpp>
#include "kalman_interfaces/msg/arm_compressed.hpp"
#include "kalman_interfaces/msg/master_message.hpp"
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

std::string hex_str(uint8_t val) {
  std::stringstream ss;
  ss << std::hex << static_cast<int>(val);
  return ss.str();
}

// We'll just set up parameters here
const std::string SPACEMOUSE_TOPIC =
    "/master_com/master_to_ros/x" +
    hex_str(kalman_interfaces::msg::MasterMessage().ARM_SEND_SPACEMOUSE);
const std::string JOY_TOPIC = "/joy_compressed";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";

namespace arm_master {
class MasterToServo : public rclcpp::Node {
public:
  MasterToServo(const rclcpp::NodeOptions &options)
      : Node("servo_publisher", options) {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<kalman_interfaces::msg::ArmCompressed>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](
            const kalman_interfaces::msg::ArmCompressed::ConstSharedPtr &msg) {
          return joyCB(msg);
        });

    spacemouse_sub_ =
        this->create_subscription<kalman_interfaces::msg::MasterMessage>(
            SPACEMOUSE_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const kalman_interfaces::msg::MasterMessage::ConstSharedPtr
                       &msg) { return spacemouseCB(msg); });

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        TWIST_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ =
        this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());

    switch_input_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type");
  }

  ~MasterToServo() override {}

  void joyCB(kalman_interfaces::msg::ArmCompressed::ConstSharedPtr msg) {
    if (current_command_type_ !=
        moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG) {
      setCommandType(moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG);
    }
    // Create the messages we might publish
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    uint8_t mask = msg->joints_mask;

    uint8_t idx = 0;
    for (uint8_t i = 0; i < 6; i++) {
      joint_msg->joint_names.push_back("joint_" + std::to_string(i + 1));
      if (mask & 1) {
        joint_msg->velocities.push_back(
            (double(msg->joints_data[idx++]) / 100.0) - 1);
      } else {
        joint_msg->velocities.push_back(0.0);
      }

      mask >>= 1;
    }

    joint_msg->header.stamp = this->now();
    joint_msg->header.frame_id = "arm_link";

    joint_pub_->publish(std::move(joint_msg));
  }

  void spacemouseCB(kalman_interfaces::msg::MasterMessage::ConstSharedPtr msg) {
    // Create the messages we might publish
    if (current_command_type_ !=
        moveit_msgs::srv::ServoCommandType::Request::TWIST) {
      setCommandType(moveit_msgs::srv::ServoCommandType::Request::TWIST);
    }

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = "ee";
    twist_msg->twist.linear.x = convert_data_to_spacenav(msg->data[0]);
    twist_msg->twist.linear.y = convert_data_to_spacenav(msg->data[1]);
    twist_msg->twist.linear.z = convert_data_to_spacenav(msg->data[2]);
    twist_msg->twist.angular.x = convert_data_to_spacenav(msg->data[3]);
    twist_msg->twist.angular.y = convert_data_to_spacenav(msg->data[4]);
    twist_msg->twist.angular.z = convert_data_to_spacenav(msg->data[5]);

    twist_pub_->publish(std::move(twist_msg));
  }

  void
  setCommandType(moveit_msgs::srv::ServoCommandType::Request::_command_type_type
                     command_type) {
    request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request_->command_type = command_type;
    if (switch_input_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Changing command type to: " << command_type);
      auto result = switch_input_->async_send_request(request_);
      current_command_type_ = command_type;
    }
  }

  double convert_data_to_spacenav(int data) {
    return (double(data) / 100) - 1.0;
  }

private:
  rclcpp::Subscription<kalman_interfaces::msg::ArmCompressed>::SharedPtr
      joy_sub_;
  rclcpp::Subscription<kalman_interfaces::msg::MasterMessage>::SharedPtr
      spacemouse_sub_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
  std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request_;
  moveit_msgs::srv::ServoCommandType::Request::_command_type_type
      current_command_type_ = -1;
}; // class MasterToServo

} // namespace arm_master

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_master::MasterToServo)
