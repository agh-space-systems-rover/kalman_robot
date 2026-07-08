#pragma once

#include <QWidget>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSpinBox;

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

private:
	using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

	void handle_serial_tx(const UInt8MultiArray &message);
	void handle_response_frame(const std::vector<uint8_t> &frame);

	QWidget        *widget_       = nullptr;
	QComboBox      *request_type_ = nullptr;
	QCheckBox      *arm_          = nullptr;
	QSpinBox       *stage_        = nullptr;
	QLabel         *stage_name_   = nullptr;
	QDoubleSpinBox *latitude_     = nullptr;
	QDoubleSpinBox *longitude_    = nullptr;
	QDoubleSpinBox *altitude_     = nullptr;
	QDoubleSpinBox *radius_       = nullptr;
	QLabel         *status_       = nullptr;
	QLabel         *response_     = nullptr;

	QWidget *arm_row_       = nullptr;
	QWidget *stage_row_     = nullptr;
	QWidget *latitude_row_  = nullptr;
	QWidget *longitude_row_ = nullptr;
	QWidget *altitude_row_  = nullptr;
	QWidget *radius_row_    = nullptr;

	rclcpp::Publisher<UInt8MultiArray>::SharedPtr    serial_rx_pub_;
	rclcpp::Subscription<UInt8MultiArray>::SharedPtr serial_tx_sub_;

	std::vector<uint8_t> response_frame_;
};

} // namespace kalman_arc
