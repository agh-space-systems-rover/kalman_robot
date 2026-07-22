#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <netdb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/tcp.h>

namespace {

void enable_keepalive(int fd) {
    const int enabled  = 1;
    const int idle_s   = 20;
    const int interval = 5;
    const int probes   = 3;

    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_s, sizeof(idle_s)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval)) < 0 ||
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

	for (auto *address = addresses; address != nullptr;
	     address       = address->ai_next) {
		const int fd = socket(
		    address->ai_family, address->ai_socktype, address->ai_protocol
		);
		if (fd < 0) {
			continue;
		}
		if (connect(fd, address->ai_addr, address->ai_addrlen) == 0) {
            enable_keepalive(fd);
			fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
			freeaddrinfo(addresses);
			return fd;
		}
		close(fd);
	}

	freeaddrinfo(addresses);
	throw std::runtime_error("failed to connect TCP socket");
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
		if (rc < 0 &&
		    (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
			continue;
		}
		throw std::runtime_error(
		    "tcp write: " + std::string(std::strerror(errno))
		);
	}
}

} // namespace

class RscpTcpClient final : public rclcpp::Node {
	using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

public:
	RscpTcpClient() : Node("rscp_tcp_client") {
		const auto host = declare_parameter<std::string>("host", "127.0.0.1");
		const auto port = declare_parameter<int>("port", 5555);
		socket_fd_      = connect_tcp(host, std::to_string(port));

		serial_rx_pub_ = create_publisher<UInt8MultiArray>(
		    "rscp/serial/rx_from_gs", rclcpp::SensorDataQoS()
		);
		serial_tx_sub_ = create_subscription<UInt8MultiArray>(
		    "rscp/serial/tx_from_gs",
		    rclcpp::SensorDataQoS(),
		    [this](const UInt8MultiArray::SharedPtr message) {
			    write_all(socket_fd_, message->data);
		    }
		);
		poll_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {
			read_socket();
		});

		RCLCPP_INFO(
		    get_logger(),
		    "RSCP TCP client connected to %s:%ld",
		    host.c_str(),
		    port
		);
	}

	~RscpTcpClient() override {
		if (socket_fd_ >= 0) {
			close(socket_fd_);
		}
	}

private:
	void read_socket() {
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
				throw std::runtime_error("TCP server closed the connection");
			}
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				return;
			}
			throw std::runtime_error(
			    "tcp read: " + std::string(std::strerror(errno))
			);
		}
	}

	int socket_fd_ = -1;

	rclcpp::Publisher<UInt8MultiArray>::SharedPtr    serial_rx_pub_;
	rclcpp::Subscription<UInt8MultiArray>::SharedPtr serial_tx_sub_;
	rclcpp::TimerBase::SharedPtr                     poll_timer_;

	std::vector<uint8_t> frame_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RscpTcpClient>());
	rclcpp::shutdown();
	return 0;
}
