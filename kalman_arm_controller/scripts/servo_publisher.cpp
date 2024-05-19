#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
// #include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include "kalman_interfaces/msg/master_message.hpp"
#include <format>

// We'll just set up parameters here
const std::string SPACEMOUSE_TOPIC =
    "/master_com/master_to_ros/x" + std::format("{:x}", kalman_interfaces::msg::MasterMessage().ARM_SEND_SPACEMOUSE);
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";

namespace arm_master
{
class MasterToServo : public rclcpp::Node
{
public:
  MasterToServo(const rclcpp::NodeOptions& options) : Node("servo_publisher", options)
  {
    // Setup pub/sub
    // joy_sub_ = this->create_subscription<kalman_interfaces::msg::MasterMessage>(
    //     JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
    //     [this](const kalman_interfaces::msg::MasterMessage::ConstSharedPtr& msg) { return spacemouseCB(msg); });

    spacemouse_sub_ = this->create_subscription<kalman_interfaces::msg::MasterMessage>(
        SPACEMOUSE_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const kalman_interfaces::msg::MasterMessage::ConstSharedPtr& msg) { return spacemouseCB(msg); });

    // joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    switch_input_ = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
  }

  ~MasterToServo() override
  {
  }

  void joyCB(kalman_interfaces::msg::MasterMessage::ConstSharedPtr msg)
  {
    // Create the messages we might publish
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    for (uint8_t i = 0; i < 6; i++)
    {
      joint_msg->joint_names.push_back("joint_" + std::to_string(i + 1));
      joint_msg->velocities.push_back(double(msg->data[i] - 128) / 128.0);
    }
  }

  void spacemouseCB(kalman_interfaces::msg::MasterMessage::ConstSharedPtr msg)
  {
    // Create the messages we might publish
    if (current_command_type_ != moveit_msgs::srv::ServoCommandType::Request::TWIST)
    {
      setCommandType(moveit_msgs::srv::ServoCommandType::Request::TWIST);
    }

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = "ee";
    twist_msg->twist.linear.x = double(msg->data[0] - 128) / 128.0;
    twist_msg->twist.linear.y = double(msg->data[1] - 128) / 128.0;
    twist_msg->twist.linear.z = double(msg->data[2] - 128) / 128.0;
    twist_msg->twist.angular.x = double(msg->data[3] - 128) / 128.0;
    twist_msg->twist.angular.y = double(msg->data[4] - 128) / 128.0;
    twist_msg->twist.angular.z = double(msg->data[5] - 128) / 128.0;

    twist_pub_->publish(std::move(twist_msg));
  }

  void setCommandType(moveit_msgs::srv::ServoCommandType::Request::_command_type_type command_type)
  {
    request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request_->command_type = command_type;
    if (switch_input_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Changing command type to: " << command_type);
      auto result = switch_input_->async_send_request(request_);
      current_command_type_ = command_type;
    }
  }

private:
  rclcpp::Subscription<kalman_interfaces::msg::MasterMessage>::SharedPtr joy_sub_;
  rclcpp::Subscription<kalman_interfaces::msg::MasterMessage>::SharedPtr spacemouse_sub_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
  std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request_;
  moveit_msgs::srv::ServoCommandType::Request::_command_type_type current_command_type_ = -1;
};  // class MasterToServo

}  // namespace arm_master

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_master::MasterToServo)
