#pragma once

#include <QString>
#include <QWidget>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QLabel;
class QPushButton;
class QSpinBox;
class QTimer;

namespace kalman_arc {

class RqtRscpOverTcp final : public rqt_gui_cpp::Plugin {
	Q_OBJECT

public:
	RqtRscpOverTcp();

	void initPlugin(qt_gui_cpp::PluginContext &context) override;
	void shutdownPlugin() override;
	void saveSettings(
	    qt_gui_cpp::Settings &plugin_settings,
	    qt_gui_cpp::Settings &instance_settings
	) const override;
	void restoreSettings(
	    const qt_gui_cpp::Settings &plugin_settings,
	    const qt_gui_cpp::Settings &instance_settings
	) override;

private slots:
	void update_visible_fields();
	void update_stage_name(int stage);
	void publish_request();
	void reconnect_tcp();
	void update_response_label();

private:
	using Bool            = std_msgs::msg::Bool;
	using String          = std_msgs::msg::String;
	using Trigger         = std_srvs::srv::Trigger;
	using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

	void handle_serial_tx(const UInt8MultiArray &message);
	void handle_response_frame(const std::vector<uint8_t> &frame);
	void update_connection_bar(bool connected);
	void update_connection_status(const QString &status);

	QWidget        *widget_             = nullptr;
	QComboBox      *request_type_       = nullptr;
	QCheckBox      *arm_                = nullptr;
	QSpinBox       *stage_              = nullptr;
	QLabel         *stage_name_         = nullptr;
	QDoubleSpinBox *latitude_           = nullptr;
	QDoubleSpinBox *longitude_          = nullptr;
	QDoubleSpinBox *altitude_           = nullptr;
	QDoubleSpinBox *radius_             = nullptr;
	QLabel         *status_             = nullptr;
	QLabel         *response_           = nullptr;
	QTimer         *response_age_timer_ = nullptr;
	QFrame         *connection_bar_     = nullptr;
	QLabel         *connection_label_   = nullptr;
	QPushButton    *reconnect_button_   = nullptr;

	QWidget *arm_row_       = nullptr;
	QWidget *stage_row_     = nullptr;
	QWidget *latitude_row_  = nullptr;
	QWidget *longitude_row_ = nullptr;
	QWidget *altitude_row_  = nullptr;
	QWidget *radius_row_    = nullptr;

	rclcpp::Publisher<UInt8MultiArray>::SharedPtr    serial_rx_pub_;
	rclcpp::Subscription<UInt8MultiArray>::SharedPtr serial_tx_sub_;
	rclcpp::Subscription<Bool>::SharedPtr            connected_sub_;
	rclcpp::Subscription<String>::SharedPtr          status_sub_;
	rclcpp::Client<Trigger>::SharedPtr               reconnect_client_;

	std::vector<uint8_t> response_frame_;
	QString              latest_response_text_;
	qint64               latest_response_time_ms_ = -1;
};

} // namespace kalman_arc
