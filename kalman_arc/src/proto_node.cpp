#include <chrono>
#include <cstdint>
#include <functional>
#include <kalman_interfaces/msg/detail/arc_rscp_response__struct.hpp>
#include <memory>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <vector>

#include "kalman_arc/cobs.hpp"
#include "proto/rscp.pb.h"
#include <kalman_interfaces/msg/arc_rscp_request.hpp>
#include <kalman_interfaces/msg/arc_rscp_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace {
    std::optional<rscp::RoverState> convert_rover_state(int32_t rover_state){
        switch (rover_state) {
            case kalman_interfaces::msg::ArcRscpResponse::ROVER_STATE_DISARMED: {
                return rscp::RoverState::DISARMED;
            }
            case kalman_interfaces::msg::ArcRscpResponse::ROVER_STATE_AUTONOMOUS: {
                return rscp::RoverState::AUTONOMOUS;
            }
            case kalman_interfaces::msg::ArcRscpResponse::ROVER_STATE_MANUAL: {
                return rscp::RoverState::MANUAL;
            }
        }
        return {};
    }
}

class RscpProtoNode final : public rclcpp::Node {
	using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
	using ArcRscpRequest  = kalman_interfaces::msg::ArcRscpRequest;
	using ArcRscpResponse = kalman_interfaces::msg::ArcRscpResponse;

public:
	RscpProtoNode() : Node("rscp_proto_node") {
		serial_rx_sub_ = create_subscription<UInt8MultiArray>(
		    "rscp/serial/rx",
		    rclcpp::SensorDataQoS(),
		    [this](const UInt8MultiArray::SharedPtr msg) {
			    this->handle_serial_rx(
			        *msg
			    ); // TODO: find out if lambda or bind is the current idiomatic
			       // way to do this
		    }
		);

		serial_tx_pub_ = create_publisher<UInt8MultiArray>(
		    "rscp/serial/tx", rclcpp::SystemDefaultsQoS()
		);

		req_pub_ = create_publisher<ArcRscpRequest>("rscp/req", 10);
		res_sub_ = create_subscription<ArcRscpResponse>(
		    "rscp/res",
		    10,
		    std::bind(&RscpProtoNode::res_callback, this, std::placeholders::_1)
		);

		RCLCPP_INFO(get_logger(), "RSCP protobuf node started");
	}

private:
	void handle_serial_rx(const UInt8MultiArray &msg) {
		for (auto byte : msg.data) {
			if (byte == 0x00) {
				// Process the complete frame
				parse_message(buffer);
				buffer.clear();
			} else {
				// Accumulate bytes into the current frame
				buffer.push_back(byte);
			}
		}
	}

	void parse_message(const std::vector<uint8_t> &buf) {
		auto                  decoded = cobs_decode(buf.data(), buf.size());
		rscp::RequestEnvelope request;
		if (!request.ParseFromArray(decoded.data(), decoded.size())) {
			RCLCPP_ERROR(this->get_logger(), "Failed to parse RSCP message");
			return;
		}
		RCLCPP_INFO_STREAM(
		    get_logger(), "Got RSCP message: " << request.DebugString()
		);

		ArcRscpRequest rscp_request;
		rscp_request.type = -1; // Mark NULL case
		switch (request.request_case()) {
		case rscp::RequestEnvelope::kArmDisarm: {
			rscp_request.type = ArcRscpRequest::ARM_DISARM;
			rscp_request.arm  = request.arm_disarm().value();
			break;
		}
		case rscp::RequestEnvelope::kSetStage: {
			rscp_request.type  = ArcRscpRequest::SET_STAGE;
			rscp_request.stage = request.set_stage().value();
			break;
		}
		case rscp::RequestEnvelope::kNavigateToGps: {
			rscp_request.type      = ArcRscpRequest::NAV_TO_GPS;
			const auto coordinate  = request.navigate_to_gps().coordinate();
			rscp_request.latitude  = coordinate.latitude();
			rscp_request.longitude = coordinate.longitude();
			break;
		}
		case rscp::RequestEnvelope::kSearchArea: {
			rscp_request.type      = ArcRscpRequest::SEARCH_AREA;
			const auto coordinate  = request.search_area().center_coordinate();
			rscp_request.latitude  = coordinate.latitude();
			rscp_request.longitude = coordinate.longitude();
			rscp_request.radius    = request.search_area().radius();
			break;
		}
		case rscp::RequestEnvelope::kStartExploration: {
			rscp_request.type = ArcRscpRequest::START_EXPLORATION;
			break;
		}
		case rscp::RequestEnvelope::REQUEST_NOT_SET:
			RCLCPP_ERROR(
			    get_logger(), "Got rscp::RequestEnvelope::REQUEST_NOT_SET"
			);
			break;
		}
		publish_arm_rscp_request(rscp_request);
	}

	void publish_arm_rscp_request(const ArcRscpRequest &rscp_request) {
		this->req_pub_->publish(rscp_request);
	}

	void res_callback(const ArcRscpResponse &ros_response) {
		rscp::ResponseEnvelope response;
		switch (ros_response.type) {
		case ArcRscpResponse::ACK: {
			std::ignore =
			    response.mutable_acknowledge(); // No nothing to set inside
			break;
		}
		case ArcRscpResponse::TASK_FINISHED: {
			rscp::ResponseEnvelope response;
			std::ignore =
			    response.mutable_task_finished(); // No nothing to set inside
			break;
		}
		case ArcRscpResponse::GPS_COORDINATE: {
    		rscp::ResponseEnvelope response;
    		auto gps_coordinate = response.mutable_gps_coordinate();
            gps_coordinate->set_latitude(ros_response.latitude);
            gps_coordinate->set_longitude(ros_response.longitude);
            gps_coordinate->set_altitude(ros_response.altitude);
    		break;
		}
		case ArcRscpResponse::DISTANCE: {
    		rscp::ResponseEnvelope response;
            response.set_distance(ros_response.distance);
    		break;
		}
		case ArcRscpResponse::MESSAGE: {
    		rscp::ResponseEnvelope response;
            response.set_message(ros_response.message);
    		break;
		}
		case ArcRscpResponse::ROVER_STATUS: {
    		rscp::ResponseEnvelope response;
            auto rover_status = response.mutable_rover_status();
            auto rover_status_opt = convert_rover_state(ros_response.rover_state);
            if (!rover_status_opt.has_value()){
                RCLCPP_ERROR( this->get_logger(), "Rover status %d is invalid", ros_response.rover_state);
                return;
            }
            rover_status->set_state(rover_status_opt.value());

            auto gps_coordinate = rover_status->coordinate();
            gps_coordinate.set_altitude(ros_response.altitude);
            gps_coordinate.set_longitude(ros_response.longitude);
            gps_coordinate.set_latitude(ros_response.latitude);

            rover_status->set_heading(ros_response.heading);

            auto battery_state = rover_status->mutable_battery_state();
            battery_state->set_voltage(ros_response.voltage);
            battery_state->set_current(ros_response.current);
            battery_state->set_state_of_charge(ros_response.state_of_charge);
    		break;
		}
		default: {
			RCLCPP_ERROR(
			    this->get_logger(),
			    "RSCP response type: %d not implemented",
			    ros_response.type
			);
			return;
		}
		}
		send_rscp_reponse(response);
	}

	void send_rscp_reponse(const rscp::ResponseEnvelope &response) {
		RCLCPP_INFO_STREAM(
		    get_logger(), "Sending RSCP message: " << response.DebugString()
		);
		std::vector<uint8_t> bytes(response.ByteSizeLong());
		if (!response.SerializeToArray(bytes.data(), bytes.size())){
    		RCLCPP_ERROR( this->get_logger(), "Failed to serialize protobuf message" );
            return;
		}
		std::vector framed_bytes = cobs_encode(bytes.data(), bytes.size());
		framed_bytes.push_back(0);  // Null terminate as in rscp repo examples
		UInt8MultiArray msg;
		msg.data = std::move(framed_bytes);
		serial_tx_pub_->publish(msg);
	}

	rclcpp::Subscription<UInt8MultiArray>::SharedPtr serial_rx_sub_;
	rclcpp::Publisher<UInt8MultiArray>::SharedPtr    serial_tx_pub_;

	rclcpp::Publisher<ArcRscpRequest>::SharedPtr     req_pub_;
	rclcpp::Subscription<ArcRscpResponse>::SharedPtr res_sub_;

	std::vector<uint8_t> buffer;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<RscpProtoNode>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
