#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "kalman_interfaces/msg/arm_compressed.hpp"
#include "kalman_interfaces/msg/master_message.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <example_interfaces/msg/empty.hpp>

std::string hex_str(uint8_t val) {
    std::stringstream ss;
    ss << std::hex << static_cast<int>(val);
    return ss.str();
}

const std::string SPACEMOUSE_TOPIC =
    "/master_com/master_to_ros/x" +
    hex_str(kalman_interfaces::msg::MasterMessage().ARM_SEND_SPACEMOUSE);
const std::string JOY_TOPIC              = "/joy_compressed";
const std::string TWIST_TOPIC            = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC            = "/servo_node/delta_joint_cmds";
const std::string CONTROL_TOPIC          = "/change_control_type";
const std::string POSE_ABORT_TOPIC       = "/pose_request/abort";
const std::string TRAJECTORY_ABORT_TOPIC = "/trajectory/abort";

namespace arm_master {
class MasterToServo : public rclcpp::Node {
public:
    MasterToServo(const rclcpp::NodeOptions &options)
        : Node("servo_publisher", options) {
        
        joy_sub_ = this->create_subscription<kalman_interfaces::msg::ArmCompressed>(
            JOY_TOPIC,
            rclcpp::SystemDefaultsQoS(),
            [this](const kalman_interfaces::msg::ArmCompressed::ConstSharedPtr &msg) {
                joyCB(msg);
            }
        );

        spacemouse_sub_ = this->create_subscription<kalman_interfaces::msg::MasterMessage>(
            SPACEMOUSE_TOPIC,
            rclcpp::SystemDefaultsQoS(),
            [this](const kalman_interfaces::msg::MasterMessage::ConstSharedPtr &msg) {
                spacemouseCB(msg);
            }
        );

        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            JOINT_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            TWIST_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        control_type_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            CONTROL_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        pose_abort_pub_ = this->create_publisher<example_interfaces::msg::Empty>(
            POSE_ABORT_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        trajectory_abort_pub_ = this->create_publisher<example_interfaces::msg::Empty>(
            TRAJECTORY_ABORT_TOPIC, rclcpp::SystemDefaultsQoS()
        );

        // Próba startu ServoNode (jeśli serwis istnieje)
        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        if (servo_start_client_->wait_for_service(std::chrono::milliseconds(200))) {
            servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }
    }

    ~MasterToServo() override {}

private:
    void joyCB(kalman_interfaces::msg::ArmCompressed::ConstSharedPtr msg) {
        control_type_pub_->publish(std_msgs::msg::UInt8().set__data(1));
        trajectory_abort_pub_->publish(example_interfaces::msg::Empty());
        pose_abort_pub_->publish(example_interfaces::msg::Empty());

        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
        uint8_t mask = msg->joints_mask;
        uint8_t idx = 0;

        for (uint8_t i = 0; i < 6; i++) {
            joint_msg->joint_names.push_back("arm_joint_" + std::to_string(i + 1));
            if ((mask & 1) && msg->joints_data.size() > idx) {
                joint_msg->velocities.push_back((double(msg->joints_data[idx++]) / 100.0) - 1.0);
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
        control_type_pub_->publish(std_msgs::msg::UInt8().set__data(1));
        trajectory_abort_pub_->publish(example_interfaces::msg::Empty());
        pose_abort_pub_->publish(example_interfaces::msg::Empty());

        if (msg->data.size() < 6) return;

        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "arm_link_end";
        twist_msg->twist.linear.x  = convert_data_to_spacenav(msg->data[0]);
        twist_msg->twist.linear.y  = convert_data_to_spacenav(msg->data[1]);
        twist_msg->twist.linear.z  = convert_data_to_spacenav(msg->data[2]);
        twist_msg->twist.angular.x = convert_data_to_spacenav(msg->data[3]);
        twist_msg->twist.angular.y = convert_data_to_spacenav(msg->data[4]);
        twist_msg->twist.angular.z = convert_data_to_spacenav(msg->data[5]);

        twist_pub_->publish(std::move(twist_msg));
    }

    double convert_data_to_spacenav(int data) {
        return (double(data) / 100.0) - 1.0;
    }

    rclcpp::Subscription<kalman_interfaces::msg::ArmCompressed>::SharedPtr joy_sub_;
    rclcpp::Subscription<kalman_interfaces::msg::MasterMessage>::SharedPtr spacemouse_sub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_type_pub_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr pose_abort_pub_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr trajectory_abort_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};
} // namespace arm_master

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_master::MasterToServo>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}