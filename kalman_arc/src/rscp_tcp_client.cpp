#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/socket.h>
#include <unistd.h>

namespace {

void enable_keepalive(int fd) {
	const int enabled  = 1;
	const int idle_s   = 20;
	const int interval = 5;
	const int probes   = 3;

	if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled)) <
	        0 ||
	    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_s, sizeof(idle_s)) <
	        0 ||
	    setsockopt(
	        fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval)
	    ) < 0 ||
	    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &probes, sizeof(probes)) < 0) {
		throw std::runtime_error(
		    "configure TCP keepalive: " + std::string(std::strerror(errno))
		);
	}
}

int connect_tcp(const std::string &host, const std::string &port) {
	addrinfo hints{};
	hints.ai_family   = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	addrinfo *addresses = nullptr;
	const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &addresses);
	if (rc != 0) {
		throw std::runtime_error(gai_strerror(rc));
	}

	std::string last_error = "failed to connect TCP socket";
	for (auto *address = addresses; address != nullptr;
	     address       = address->ai_next) {
		const int fd = socket(
		    address->ai_family, address->ai_socktype, address->ai_protocol
		);
		if (fd < 0) {
			last_error = std::strerror(errno);
			continue;
		}
		if (connect(fd, address->ai_addr, address->ai_addrlen) == 0) {
			enable_keepalive(fd);
			fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
			freeaddrinfo(addresses);
			return fd;
		}
		last_error = std::strerror(errno);
		close(fd);
	}

	freeaddrinfo(addresses);
	throw std::runtime_error("failed to connect TCP socket: " + last_error);
}

void write_all(int fd, const std::vector<uint8_t> &bytes) {
	size_t written = 0;
	while (written < bytes.size()) {
		const auto rc =
		    write(fd, bytes.data() + written, bytes.size() - written);
		if (rc > 0) {
			written += static_cast<size_t>(rc);
			continue;
		}
		if (rc < 0 && errno == EINTR) {
			continue;
		}
		if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			pollfd pfd{};
			pfd.fd     = fd;
			pfd.events = POLLOUT;

			const int poll_rc = poll(&pfd, 1, 1000);
			if (poll_rc > 0) {
				if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
					break;
				}
				continue;
			}
			if (poll_rc == 0) {
				throw std::runtime_error("tcp write timeout");
			}
			if (errno == EINTR) {
				continue;
			}
		}
		throw std::runtime_error(
		    "tcp write: " + std::string(std::strerror(errno))
		);
	}
}

} // namespace

class RscpTcpClient final : public rclcpp::Node {
	using Bool            = std_msgs::msg::Bool;
	using String          = std_msgs::msg::String;
	using Trigger         = std_srvs::srv::Trigger;
	using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

public:
	RscpTcpClient() : Node("rscp_tcp_client") {
		host_ = declare_parameter<std::string>("host", "127.0.0.1");
		port_ = declare_parameter<int>("port", 5555);

		serial_rx_pub_ = create_publisher<UInt8MultiArray>(
		    "rscp/tcp/rx_from_gs", rclcpp::SensorDataQoS()
		);
		const auto state_qos = rclcpp::QoS(1).transient_local();
		connected_pub_ =
		    create_publisher<Bool>("rscp/tcp/connected", state_qos);
		status_pub_ = create_publisher<String>("rscp/tcp/status", state_qos);

		serial_tx_sub_ = create_subscription<UInt8MultiArray>(
		    "rscp/tcp/tx_from_gs",
		    rclcpp::SensorDataQoS(),
		    [this](const UInt8MultiArray::SharedPtr message) {
			    write_socket(message->data);
		    }
		);
		reconnect_srv_ = create_service<Trigger>(
		    "rscp/tcp/reconnect",
		    [this](
		        const std::shared_ptr<Trigger::Request>,
		        const std::shared_ptr<Trigger::Response> response
		    ) {
			    disconnect("manual reconnect requested");

			    std::string error;
			    response->success = connect_socket(&error);
			    response->message = response->success ? "connected" : error;
		    }
		);
		poll_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {
			read_socket();
		});

		std::string error;
		if (!connect_socket(&error)) {
			RCLCPP_WARN(
			    get_logger(),
			    "RSCP TCP client disconnected from %s:%d: %s",
			    host_.c_str(),
			    port_,
			    error.c_str()
			);
		}
	}

	~RscpTcpClient() override {
		close_socket();
	}

private:
	void publish_connection(bool connected, const std::string &status) {
		Bool connected_msg;
		connected_msg.data = connected;
		connected_pub_->publish(connected_msg);

		String status_msg;
		status_msg.data = status;
		status_pub_->publish(status_msg);
	}

	void close_socket() {
		if (socket_fd_ >= 0) {
			close(socket_fd_);
			socket_fd_ = -1;
		}
	}

	void disconnect(const std::string &reason) {
		const bool was_connected = socket_fd_ >= 0;
		close_socket();
		frame_.clear();
		publish_connection(false, reason);
		if (was_connected) {
			RCLCPP_WARN(
			    get_logger(), "RSCP TCP disconnected: %s", reason.c_str()
			);
		}
	}

	bool connect_socket(std::string *error_out) {
		try {
			socket_fd_ = connect_tcp(host_, std::to_string(port_));
			publish_connection(true, "connected");
			RCLCPP_INFO(
			    get_logger(),
			    "RSCP TCP client connected to %s:%d",
			    host_.c_str(),
			    port_
			);
			return true;
		} catch (const std::exception &error) {
			close_socket();
			if (error_out != nullptr) {
				*error_out = error.what();
			}
			publish_connection(false, error.what());
			return false;
		}
	}

	void write_socket(const std::vector<uint8_t> &bytes) {
		if (socket_fd_ < 0) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    2000,
			    "Dropping RSCP request: TCP disconnected"
			);
			return;
		}

		try {
			write_all(socket_fd_, bytes);
		} catch (const std::exception &error) {
			disconnect(error.what());
		}
	}

	void read_socket() {
		if (socket_fd_ < 0) {
			return;
		}

		uint8_t buffer[512];
		while (true) {
			const auto count = read(socket_fd_, buffer, sizeof(buffer));
			if (count > 0) {
				for (ssize_t i = 0; i < count; ++i) {
					frame_.push_back(buffer[i]);
					if (buffer[i] == 0x00) {
						UInt8MultiArray message;
						message.data = std::move(frame_);
						serial_rx_pub_->publish(message);
						frame_.clear();
					}
				}
				continue;
			}
			if (count == 0) {
				disconnect("TCP server closed the connection");
				return;
			}
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				return;
			}
			disconnect("tcp read: " + std::string(std::strerror(errno)));
			return;
		}
	}

	std::string host_;
	int         port_      = 5555;
	int         socket_fd_ = -1;

	rclcpp::Publisher<UInt8MultiArray>::SharedPtr    serial_rx_pub_;
	rclcpp::Subscription<UInt8MultiArray>::SharedPtr serial_tx_sub_;
	rclcpp::Publisher<Bool>::SharedPtr               connected_pub_;
	rclcpp::Publisher<String>::SharedPtr             status_pub_;
	rclcpp::Service<Trigger>::SharedPtr              reconnect_srv_;
	rclcpp::TimerBase::SharedPtr                     poll_timer_;

	std::vector<uint8_t> frame_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RscpTcpClient>());
	rclcpp::shutdown();
	return 0;
}
