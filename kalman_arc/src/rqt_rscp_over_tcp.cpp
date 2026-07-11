#include "kalman_arc/rqt_rscp_over_tcp.hpp"

#include "kalman_arc/cobs.hpp"
#include "proto/rscp.pb.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
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

	auto *publish = new QPushButton("Publish to rscp/serial/rx");
	layout->addWidget(publish);
	status_ = new QLabel("Ready");
	layout->addWidget(status_);

	response_ = new QLabel("No response received from rscp/serial/tx");
	response_->setWordWrap(true);
	response_->setTextInteractionFlags(Qt::TextSelectableByMouse);
	layout->addWidget(response_);
	layout->addStretch();

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

	serial_rx_pub_ =
	    node_->create_publisher<UInt8MultiArray>("rscp/serial/tx_to_raspi", 10);
	serial_tx_sub_ = node_->create_subscription<UInt8MultiArray>(
	    "rscp/serial/rx_to_raspi",
	    rclcpp::SensorDataQoS(),
	    [this](const UInt8MultiArray::SharedPtr message) {
		    handle_serial_tx(*message);
	    }
	);

	update_stage_name(stage_->value());
	update_visible_fields();
	context.addWidget(widget_);
}

void RqtRscpOverTcp::shutdownPlugin() {
	serial_rx_pub_.reset();
	serial_tx_sub_.reset();
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
		        "Published COBS-framed request to rscp/serial/rx:\n%1"
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
	    [label = response_, text]() {
		    label->setText(
		        QStringLiteral("Latest response from rscp/serial/tx:\n%1")
		            .arg(text)
		    );
	    },
	    Qt::QueuedConnection
	);
}

} // namespace kalman_arc

PLUGINLIB_EXPORT_CLASS(kalman_arc::RqtRscpOverTcp, rqt_gui_cpp::Plugin)
