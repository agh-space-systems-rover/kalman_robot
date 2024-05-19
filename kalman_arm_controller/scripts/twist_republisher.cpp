#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class TwistRepublisher : public rclcpp::Node
{
public:
    TwistRepublisher() : Node("twist_republisher")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spacenav/twist", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                republishTwist(msg);
            });

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
    }

private:
    void republishTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        stamped_msg->header.stamp = this->now();
        stamped_msg->header.frame_id = "ee";
        stamped_msg->twist = *msg;
        publisher_->publish(*stamped_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
