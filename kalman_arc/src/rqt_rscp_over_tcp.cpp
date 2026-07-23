#include "kalman_arc/rqt_rscp_over_tcp.hpp"

#include "kalman_arc/cobs.hpp"
#include "proto/rscp.pb.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

#include <limits>
#include <stdexcept>
#include <string>

namespace {

template <typename Widget>
QWidget *add_form_row(QFormLayout *form, const QString &label, Widget *field) {
	auto *row    = new QWidget;
	auto *layout = new QFormLayout(row);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addRow(label, field);
	form->addRow(row);
	return row;
}

QString rover_state_name(rscp::RoverState state) {
	switch (state) {
	case rscp::DISARMED:
		return QStringLiteral("DISARMED");
	case rscp::AUTONOMOUS:
		return QStringLiteral("AUTONOMOUS");
	case rscp::MANUAL:
		return QStringLiteral("MANUAL");
	default:
		return QStringLiteral("Unknown (%1)").arg(static_cast<int>(state));
	}
}

QString format_coordinate(const rscp::GPSCoordinate &coordinate) {
	return QStringLiteral(
	           "Latitude: %1 deg\n"
	           "Longitude: %2 deg\n"
	           "Altitude: %3 m"
	)
	    .arg(coordinate.latitude(), 0, 'f', 8)
	    .arg(coordinate.longitude(), 0, 'f', 8)
	    .arg(coordinate.altitude(), 0, 'f', 3);
}

QString format_response(const rscp::ResponseEnvelope &response) {
	switch (response.response_case()) {
	case rscp::ResponseEnvelope::kAcknowledge:
		return QStringLiteral("Acknowledge");
	case rscp::ResponseEnvelope::kTaskFinished:
		return QStringLiteral("Task finished");
	case rscp::ResponseEnvelope::kGpsCoordinate:
		return QStringLiteral("GPS coordinate\n%1")
		    .arg(format_coordinate(response.gps_coordinate()));
	case rscp::ResponseEnvelope::kDistance:
		return QStringLiteral("Distance: %1 m")
		    .arg(response.distance(), 0, 'f', 3);
	case rscp::ResponseEnvelope::kMessage:
		return QStringLiteral("Message\n%1")
		    .arg(QString::fromStdString(response.message()));
	case rscp::ResponseEnvelope::kRoverStatus: {
		const auto &status  = response.rover_status();
		const auto &battery = status.battery_state();
		return QStringLiteral(
		           "Rover status\n"
		           "State: %1\n"
		           "%2\n"
		           "Heading: %3\n"
		           "Battery voltage: %4 V\n"
		           "Battery current: %5 A\n"
		           "Battery state of charge: %6"
		)
		    .arg(rover_state_name(status.state()))
		    .arg(format_coordinate(status.coordinate()))
		    .arg(status.heading(), 0, 'f', 3)
		    .arg(battery.voltage(), 0, 'f', 3)
		    .arg(battery.current(), 0, 'f', 3)
		    .arg(battery.state_of_charge(), 0, 'f', 3);
	}
	case rscp::ResponseEnvelope::RESPONSE_NOT_SET:
		return QStringLiteral("Empty response envelope");
	default:
		return QStringLiteral("Unknown response");
	}
}

std::vector<uint8_t> serialize_frame(const rscp::RequestEnvelope &request) {
	std::vector<uint8_t> bytes(request.ByteSizeLong());
	if (!request.SerializeToArray(bytes.data(), bytes.size())) {
		throw std::runtime_error("Failed to serialize protobuf request");
	}

	auto framed = cobs_encode(bytes.data(), bytes.size());
	framed.push_back(0);
	return framed;
}

QString format_response_age(qint64 elapsed_ms) {
	const auto elapsed_s = elapsed_ms / 1000;
	if (elapsed_s < 60) {
		return QStringLiteral("%1 s ago").arg(elapsed_s);
	}

	const auto elapsed_min = elapsed_s / 60;
	if (elapsed_min < 60) {
		return QStringLiteral("%1 min %2 s ago")
		    .arg(elapsed_min)
		    .arg(elapsed_s % 60);
	}

	return QStringLiteral("%1 h %2 min ago")
	    .arg(elapsed_min / 60)
	    .arg(elapsed_min % 60);
}

} // namespace

namespace kalman_arc {

RqtRscpOverTcp::RqtRscpOverTcp() {
	setObjectName("RqtRscpOverTcp");
}

void RqtRscpOverTcp::initPlugin(qt_gui_cpp::PluginContext &context) {
	widget_      = new QWidget;
	auto *layout = new QVBoxLayout(widget_);
	auto *form   = new QFormLayout;
	layout->addLayout(form);

	request_type_ = new QComboBox;
	request_type_->addItems(
	    {"Arm/disarm",
	     "Set stage",
	     "Navigate to GPS",
	     "Search area",
	     "Start exploration"}
	);
	form->addRow("Request", request_type_);

	arm_     = new QCheckBox("Arm");
	arm_row_ = add_form_row(form, "Arm/disarm", arm_);

	stage_ = new QSpinBox;
	stage_->setRange(0, std::numeric_limits<int>::max());
	stage_name_        = new QLabel;
	auto *stage_widget = new QWidget;
	auto *stage_layout = new QVBoxLayout(stage_widget);
	stage_layout->setContentsMargins(0, 0, 0, 0);
	stage_layout->addWidget(stage_);
	stage_layout->addWidget(stage_name_);
	stage_row_ = add_form_row(form, "Stage", stage_widget);

	latitude_ = new QDoubleSpinBox;
	latitude_->setRange(-90.0, 90.0);
	latitude_->setDecimals(8);
	latitude_row_ = add_form_row(form, "Latitude", latitude_);

	longitude_ = new QDoubleSpinBox;
	longitude_->setRange(-180.0, 180.0);
	longitude_->setDecimals(8);
	longitude_row_ = add_form_row(form, "Longitude", longitude_);

	altitude_ = new QDoubleSpinBox;
	altitude_->setRange(-10000.0, 100000.0);
	altitude_->setDecimals(3);
	altitude_->setSuffix(" m");
	altitude_row_ = add_form_row(form, "Altitude", altitude_);

	radius_ = new QDoubleSpinBox;
	radius_->setRange(0.0, 1000000.0);
	radius_->setDecimals(3);
	radius_->setSuffix(" m");
	radius_row_ = add_form_row(form, "Radius", radius_);

	auto *publish = new QPushButton("Publish to rscp/tcp/tx_from_gs");
	layout->addWidget(publish);
	status_ = new QLabel("Ready");
	layout->addWidget(status_);

	response_ = new QLabel("No response received from rscp/tcp/rx_from_gs");
	response_->setWordWrap(true);
	response_->setTextInteractionFlags(Qt::TextSelectableByMouse);
	layout->addWidget(response_);

	response_age_timer_ = new QTimer(widget_);
	connect(
	    response_age_timer_,
	    &QTimer::timeout,
	    this,
	    &RqtRscpOverTcp::update_response_label
	);
	response_age_timer_->start(1000);

	layout->addStretch();

	connection_bar_ = new QFrame;
	connection_bar_->setFrameShape(QFrame::StyledPanel);
	connection_bar_->setStyleSheet(
	    "QFrame { background-color: #5c1f1f; color: white; padding: 6px; }"
	);
	auto *connection_layout = new QHBoxLayout(connection_bar_);
	connection_label_       = new QLabel("TCP connection lost");
	reconnect_button_       = new QPushButton("Reconnect");
	connection_layout->addWidget(connection_label_);
	connection_layout->addStretch();
	connection_layout->addWidget(reconnect_button_);
	connection_bar_->setVisible(false);
	layout->addWidget(connection_bar_);

	connect(
	    request_type_,
	    qOverload<int>(&QComboBox::currentIndexChanged),
	    this,
	    &RqtRscpOverTcp::update_visible_fields
	);
	connect(
	    publish, &QPushButton::clicked, this, &RqtRscpOverTcp::publish_request
	);
	connect(
	    stage_,
	    qOverload<int>(&QSpinBox::valueChanged),
	    this,
	    &RqtRscpOverTcp::update_stage_name
	);
	connect(
	    reconnect_button_,
	    &QPushButton::clicked,
	    this,
	    &RqtRscpOverTcp::reconnect_tcp
	);

	serial_rx_pub_ = node_->create_publisher<UInt8MultiArray>(
	    "rscp/tcp/tx_from_gs", 10
	);
	serial_tx_sub_ = node_->create_subscription<UInt8MultiArray>(
	    "rscp/tcp/rx_from_gs",
	    rclcpp::SensorDataQoS(),
	    [this](const UInt8MultiArray::SharedPtr message) {
		    handle_serial_tx(*message);
	    }
	);
	const auto state_qos = rclcpp::QoS(1).transient_local();
	connected_sub_       = node_->create_subscription<Bool>(
	    "rscp/tcp/connected", state_qos, [this](const Bool::SharedPtr message) {
		    QMetaObject::invokeMethod(
		        connection_bar_,
		        [this, connected = message->data]() {
			        update_connection_bar(connected);
		        },
		        Qt::QueuedConnection
		    );
	    }
	);
	status_sub_ = node_->create_subscription<String>(
	    "rscp/tcp/status", state_qos, [this](const String::SharedPtr message) {
		    QMetaObject::invokeMethod(
		        connection_bar_,
		        [this, status = QString::fromStdString(message->data)]() {
			        update_connection_status(status);
		        },
		        Qt::QueuedConnection
		    );
	    }
	);
	reconnect_client_ = node_->create_client<Trigger>("rscp/tcp/reconnect");

	update_stage_name(stage_->value());
	update_visible_fields();
	context.addWidget(widget_);
}

void RqtRscpOverTcp::shutdownPlugin() {
	serial_rx_pub_.reset();
	serial_tx_sub_.reset();
	connected_sub_.reset();
	status_sub_.reset();
	reconnect_client_.reset();
}

void RqtRscpOverTcp::saveSettings(
    qt_gui_cpp::Settings &, qt_gui_cpp::Settings &instance_settings
) const {
	instance_settings.setValue("request_type", request_type_->currentIndex());
	instance_settings.setValue("arm", arm_->isChecked());
	instance_settings.setValue("stage", stage_->value());
	instance_settings.setValue("latitude", latitude_->value());
	instance_settings.setValue("longitude", longitude_->value());
	instance_settings.setValue("altitude", altitude_->value());
	instance_settings.setValue("radius", radius_->value());
}

void RqtRscpOverTcp::restoreSettings(
    const qt_gui_cpp::Settings &, const qt_gui_cpp::Settings &instance_settings
) {
	request_type_->setCurrentIndex(
	    instance_settings.value("request_type", 0).toInt()
	);
	arm_->setChecked(instance_settings.value("arm", false).toBool());
	stage_->setValue(instance_settings.value("stage", 0).toInt());
	latitude_->setValue(instance_settings.value("latitude", 0.0).toDouble());
	longitude_->setValue(instance_settings.value("longitude", 0.0).toDouble());
	altitude_->setValue(instance_settings.value("altitude", 0.0).toDouble());
	radius_->setValue(instance_settings.value("radius", 0.0).toDouble());
	update_visible_fields();
}

void RqtRscpOverTcp::update_visible_fields() {
	const auto type = request_type_->currentIndex();
	arm_row_->setVisible(type == 0);
	stage_row_->setVisible(type == 1);
	latitude_row_->setVisible(type == 2 || type == 3);
	longitude_row_->setVisible(type == 2 || type == 3);
	altitude_row_->setVisible(type == 2 || type == 3);
	radius_row_->setVisible(type == 3);
}

void RqtRscpOverTcp::update_stage_name(int stage) {
	switch (stage) {
	case 1:
		stage_name_->setText("Antenna Installation");
		break;
	case 2:
		stage_name_->setText("Shackleton Crater");
		break;
	case 3:
		stage_name_->setText("Lava Tube");
		break;
	case 4:
		stage_name_->setText("Return to Airlock");
		break;
	default:
		stage_name_->setText("Invalid stage number");
		break;
	}
}

void RqtRscpOverTcp::publish_request() {
	rscp::RequestEnvelope request;
	switch (request_type_->currentIndex()) {
	case 0:
		request.mutable_arm_disarm()->set_value(arm_->isChecked());
		break;
	case 1:
		request.mutable_set_stage()->set_value(stage_->value());
		break;
	case 2: {
		auto *coordinate =
		    request.mutable_navigate_to_gps()->mutable_coordinate();
		coordinate->set_latitude(latitude_->value());
		coordinate->set_longitude(longitude_->value());
		coordinate->set_altitude(static_cast<float>(altitude_->value()));
		break;
	}
	case 3: {
		auto *search     = request.mutable_search_area();
		auto *coordinate = search->mutable_center_coordinate();
		coordinate->set_latitude(latitude_->value());
		coordinate->set_longitude(longitude_->value());
		coordinate->set_altitude(static_cast<float>(altitude_->value()));
		search->set_radius(static_cast<float>(radius_->value()));
		break;
	}
	case 4:
		request.mutable_start_exploration()->set_dummy_field(true);
		break;
	default:
		status_->setText("Invalid request type");
		return;
	}

	try {
		UInt8MultiArray message;
		message.data = serialize_frame(request);
		serial_rx_pub_->publish(message);
		status_->setText(
		    QStringLiteral(
		        "Published COBS-framed request to "
		        "rscp/tcp/tx_from_gs:\n%1"
		    )
		        .arg(QString::fromStdString(request.DebugString()))
		);
	} catch (const std::exception &error) {
		status_->setText(
		    QStringLiteral("Failed to publish request: %1").arg(error.what())
		);
	}
}

void RqtRscpOverTcp::handle_serial_tx(const UInt8MultiArray &message) {
	for (auto byte : message.data) {
		if (byte == 0x00) {
			handle_response_frame(response_frame_);
			response_frame_.clear();
		} else {
			response_frame_.push_back(byte);
		}
	}
}

void RqtRscpOverTcp::handle_response_frame(const std::vector<uint8_t> &frame) {
	const auto             decoded = cobs_decode(frame.data(), frame.size());
	rscp::ResponseEnvelope response;
	const QString          text =
	    response.ParseFromArray(decoded.data(), decoded.size())
	        ? format_response(response)
	        : QStringLiteral("Failed to parse RSCP response protobuf");

	QMetaObject::invokeMethod(
	    response_,
	    [this, text]() {
		    latest_response_text_    = text;
		    latest_response_time_ms_ = QDateTime::currentMSecsSinceEpoch();
		    update_response_label();
	    },
	    Qt::QueuedConnection
	);
}

void RqtRscpOverTcp::update_response_label() {
	if (latest_response_time_ms_ < 0) {
		return;
	}

	const auto elapsed_ms =
	    QDateTime::currentMSecsSinceEpoch() - latest_response_time_ms_;
	response_->setText(QStringLiteral(
	                       "Latest response from rscp/tcp/rx_from_gs "
	                       "(received %1):\n%2"
	)
	                       .arg(format_response_age(elapsed_ms))
	                       .arg(latest_response_text_));
}

void RqtRscpOverTcp::update_connection_bar(bool connected) {
	connection_bar_->setVisible(!connected);
	if (connected) {
		reconnect_button_->setEnabled(true);
		connection_label_->setText("TCP connected");
	} else if (
	    connection_label_->text().isEmpty() ||
	    connection_label_->text() == "TCP connected"
	) {
		connection_label_->setText("TCP connection lost");
	}
}

void RqtRscpOverTcp::update_connection_status(const QString &status) {
	if (connection_bar_->isVisible()) {
		connection_label_->setText(
		    QStringLiteral("TCP connection lost: %1").arg(status)
		);
	}
}

void RqtRscpOverTcp::reconnect_tcp() {
	if (!reconnect_client_->service_is_ready()) {
		connection_bar_->setVisible(true);
		connection_label_->setText("Reconnect service unavailable");
		return;
	}

	reconnect_button_->setEnabled(false);
	connection_bar_->setVisible(true);
	connection_label_->setText("Reconnecting...");

	auto request = std::make_shared<Trigger::Request>();
	reconnect_client_->async_send_request(
	    request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
		    const auto response = future.get();
		    QMetaObject::invokeMethod(
		        connection_bar_,
		        [this, response]() {
			        reconnect_button_->setEnabled(true);
			        if (response->success) {
				        connection_label_->setText("TCP connected");
				        connection_bar_->setVisible(false);
			        } else {
				        connection_label_->setText(
				            QStringLiteral("Reconnect failed: %1")
				                .arg(QString::fromStdString(response->message))
				        );
				        connection_bar_->setVisible(true);
			        }
		        },
		        Qt::QueuedConnection
		    );
	    }
	);
}

} // namespace kalman_arc

PLUGINLIB_EXPORT_CLASS(kalman_arc::RqtRscpOverTcp, rqt_gui_cpp::Plugin)
