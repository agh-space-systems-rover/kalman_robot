#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "kalman_interfaces/msg/master_message.hpp"
#include <cctype>
#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

struct Data {
	std::array<std::string, 64> arr;

	static bool skip_ix(size_t ix) {
		return ix == 0 || ix % 4 == 3;
	}

	std::string generate_string() const {
		std::string ret{};
		for (size_t ix = 0; ix < arr.size(); ix++) {
			if (skip_ix(ix)) {
				continue;
			}
			const auto &str = arr.at(ix);

			if (str.empty()) {
				ret.append("???");
			} else {
				ret.append(str);
			}
		}

		return ret;
	}
};

namespace kalman_master {

class RfidDriver : public rclcpp::Node {
	using MasterMessage = kalman_interfaces::msg::MasterMessage;

  public:
	RfidDriver(const rclcpp::NodeOptions &options)
	    : Node("rfid_driver", options) {
		master_sub_ = this->create_subscription<MasterMessage>(
		    "master_com/master_to_ros/x85",
		    10,
		    std::bind(&RfidDriver::master_callback, this, std::placeholders::_1)
		);

		pub_ = this->create_publisher<std_msgs::msg::String>("rfid_text", 10);

		RCLCPP_INFO(this->get_logger(), "rfid_node has been started.");
	}

  private:
	void master_callback(const MasterMessage::SharedPtr master_msg) {
		const auto now = std::chrono::high_resolution_clock::now();
		if (now - last_update > std::chrono::seconds(120)) {
			data_ = Data{};
		}
		last_update = now;

		if (master_msg->cmd != FRAME_ID) {
			RCLCPP_ERROR(
			    this->get_logger(), "got message with incorrect frame_id"
			);
		}
		const std::vector<uint8_t> &data = master_msg->data;
		if (data.size() < 17){
			return;
		}

		const std::string str_part = std::string(data.begin(), data.begin() + 16);
		const uint8_t     msg_id   = data.at(16);
		if (msg_id >= data_.arr.size()) {
			return;
		}
		data_.arr.at(msg_id) = str_part;

		std_msgs::msg::String ret{};
		ret.data = data_.generate_string();
		pub_->publish(ret);
	}

	rclcpp::Subscription<MasterMessage>::SharedPtr              master_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         pub_;
	Data                                                        data_;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_update;

	constexpr static auto FRAME_ID =
	    kalman_interfaces::msg::MasterMessage::RFID;
};

} // namespace kalman_master

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_master::RfidDriver)
