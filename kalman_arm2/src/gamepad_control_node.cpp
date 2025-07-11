#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/msg/arm_joint_values.hpp>
#include <functional>

namespace kalman_arm2 {

// Gamepad axis indices
constexpr int LEFT_X = 0;
constexpr int LEFT_Y = 1;
constexpr int LEFT_TRIGGER = 2;
constexpr int RIGHT_X = 3;
constexpr int RIGHT_Y = 4;
constexpr int RIGHT_TRIGGER = 5;

// Gamepad button indices
constexpr int SOUTH_BUTTON = 0;  // A button
constexpr int NORTH_BUTTON = 3;  // Y button
constexpr int LEFT_SHOULDER = 4;
constexpr int RIGHT_SHOULDER = 5;

class GamepadControl : public rclcpp::Node {
public:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
    rclcpp::Publisher<kalman_interfaces::msg::ArmJointValues>::SharedPtr jaw_pub;

    // Parameters
    double linear_scale;
    double angular_scale;
    double roll_scale;
    double jaw_vel_scale;
    std::string end_effector_frame;

    geometry_msgs::msg::TwistStamped last_twist_msg;

    GamepadControl(const rclcpp::NodeOptions &options)
        : Node("gamepad_control", options) {
        
        // Declare parameters
        this->declare_parameter<double>("linear_scale", 0.1);  // m/s
        this->declare_parameter<double>("angular_scale", 0.5); // rad/s
        this->declare_parameter<double>("roll_scale", 0.3);    // rad/s
        this->declare_parameter<double>("jaw_vel_scale", 1.0); // rad/s
        this->declare_parameter<std::string>("end_effector_frame", "arm_link_end");
        
        this->get_parameter("linear_scale", linear_scale);
        this->get_parameter("angular_scale", angular_scale);
        this->get_parameter("roll_scale", roll_scale);
        this->get_parameter("jaw_vel_scale", jaw_vel_scale);
        this->get_parameter("end_effector_frame", end_effector_frame);

        // Publishers & subscribers
        joy_sub = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&GamepadControl::on_joy, this, std::placeholders::_1));

        twist_pub = create_publisher<geometry_msgs::msg::TwistStamped>("target_twist", 10);
        jaw_pub = create_publisher<kalman_interfaces::msg::ArmJointValues>("jaw_vel", 10);

        RCLCPP_INFO(get_logger(), "Gamepad arm control node started");
    }

private:
    void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Insufficient gamepad axes or buttons");
            return;
        }

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = now();
        twist_msg.header.frame_id = end_effector_frame;

        // Left stick: controls linear in Y/Z plane
        twist_msg.twist.linear.y = msg->axes[LEFT_X] * linear_scale;
        twist_msg.twist.linear.z = msg->axes[LEFT_Y] * linear_scale;

        // Right stick: controls pitch and yaw
        twist_msg.twist.angular.y = -msg->axes[RIGHT_Y] * angular_scale;  // pitch
        twist_msg.twist.angular.z = msg->axes[RIGHT_X] * angular_scale;  // yaw

        // Triggers control linear fwd/bwd (X)
        double left_trigger_val = (-msg->axes[LEFT_TRIGGER] * 0.5 + 0.5);   // Convert from [-1,1] to [0,1]
        double right_trigger_val = (-msg->axes[RIGHT_TRIGGER] * 0.5 + 0.5); // Convert from [-1,1] to [0,1]
        twist_msg.twist.linear.x = (right_trigger_val - left_trigger_val) * linear_scale;

        // Shoulders: control roll (digital input)
        double roll_input = 0.0;
        if (msg->buttons[LEFT_SHOULDER]) {
            roll_input -= 1.0;
        }
        if (msg->buttons[RIGHT_SHOULDER]) {
            roll_input += 1.0;
        }
        twist_msg.twist.angular.x = roll_input * roll_scale;

        // Only publish if message has changed or is non-zero
        if (!twist_msg_is_zero(twist_msg) || !twist_msg_is_zero(last_twist_msg)) {
            twist_pub->publish(twist_msg);
            last_twist_msg = twist_msg;
        }

        // Jaw control with A (south) and Y (north) buttons
        double jaw_vel = 0.0;
        if (msg->buttons[SOUTH_BUTTON]) {  // A button - close jaw (negative velocity)
            jaw_vel = -jaw_vel_scale;
        }
        if (msg->buttons[NORTH_BUTTON]) {  // Y button - open jaw (positive velocity)
            jaw_vel = jaw_vel_scale;
        }

        if (jaw_vel != 0.0) {
            auto jaw_msg = kalman_interfaces::msg::ArmJointValues();
            jaw_msg.header.stamp = now();
            jaw_msg.joints.fill(0.0);  // Zero joint velocities
            jaw_msg.jaw = jaw_vel;
            jaw_pub->publish(jaw_msg);
        }
    }

    bool twist_msg_is_zero(const geometry_msgs::msg::TwistStamped &msg) {
        const auto &t = msg.twist;
        return (t.linear.x == 0.0 && t.linear.y == 0.0 && t.linear.z == 0.0 &&
                t.angular.x == 0.0 && t.angular.y == 0.0 && t.angular.z == 0.0);
    }
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::GamepadControl)
