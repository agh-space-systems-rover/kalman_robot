#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kalman_interfaces/msg/master_message.hpp"
#include <cctype>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

namespace kalman_master {

class KalmanRfidNode : public rclcpp::Node {
	using MasterMessage = kalman_interfaces::msg::MasterMessage;

  public:
	KalmanRfidNode(const rclcpp::NodeOptions &options)
	    : Node("rfid_node", options) {
		master_sub_ = this->create_subscription<MasterMessage>(
		    "master_com/master_to_ros/x85",
		    10,
		    std::bind(
		        &KalmanRfidNode::master_callback, this, std::placeholders::_1
		    )
		);

		pub_ = this->create_publisher<std_msgs::msg::String>("rfid_text", 10);

		RCLCPP_INFO(this->get_logger(), "rfid_node has been started.");
	}

  private:
	void master_callback(const MasterMessage::SharedPtr master_msg) {
		if (master_msg->cmd != FRAME_ID) {
			RCLCPP_ERROR(
			    this->get_logger(), "got message with incorrect frame_id"
			);
		}
		const std::vector<uint8_t> &data = master_msg->data;

		if (data.size() == 16) {
			RCLCPP_WARN(
			    this->get_logger(),
			    "got rfid message of incorrect size: %zu",
			    data.size()
			);
		}

		std::string str_part = std::string(data.begin(), data.end());

		std_msgs::msg::String ret{};
		ret.data = str_part;
		pub_->publish(ret);
	}

	rclcpp::Subscription<MasterMessage>::SharedPtr      master_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

	constexpr static auto FRAME_ID =
	    kalman_interfaces::msg::MasterMessage::RFID;
};

} // namespace kalman_master

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_master::KalmanRfidNode)