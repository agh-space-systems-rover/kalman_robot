#include "kalman_arc/rqt_rscp.hpp"

#include "kalman_arc/cobs.hpp"
#include "proto/rscp.pb.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QStringList>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace {

template<typename Widget>
QWidget *add_form_row(QFormLayout *form, const QString &label, Widget *field) {
	auto *row = new QWidget;
	auto *layout = new QFormLayout(row);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addRow(label, field);
	form->addRow(row);
	return row;
}

QString bytes_to_hex(const std::vector<uint8_t> &bytes) {
	QStringList values;
	for (const auto byte : bytes) {
		values.append(QStringLiteral("%1").arg(byte, 2, 16, QLatin1Char('0')));
	}
	return values.join(QLatin1Char(' '));
}

} // namespace

namespace kalman_arc {

RqtRscp::RqtRscp() {
	setObjectName("RqtRscp");
}

void RqtRscp::initPlugin(qt_gui_cpp::PluginContext &context) {
	widget_ = new QWidget;
	auto *layout = new QVBoxLayout(widget_);
	auto *form = new QFormLayout;
	layout->addLayout(form);

	request_type_ = new QComboBox;
	request_type_->addItems(
	    {"Arm/disarm", "Set stage", "Navigate to GPS", "Search area", "Start exploration"}
	);
	form->addRow("Request", request_type_);

	arm_ = new QCheckBox("Arm");
	arm_row_ = add_form_row(form, "Arm/disarm", arm_);

	stage_ = new QSpinBox;
	stage_->setRange(0, std::numeric_limits<int>::max());
	stage_name_ = new QLabel;
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
	bytes_ = new QLabel;
	bytes_->setTextInteractionFlags(Qt::TextSelectableByMouse);
	bytes_->setWordWrap(true);
	layout->addWidget(status_);
	layout->addWidget(bytes_);
	layout->addStretch();

	connect(
	    request_type_,
	    qOverload<int>(&QComboBox::currentIndexChanged),
	    this,
	    &RqtRscp::update_visible_fields
	);
	connect(publish, &QPushButton::clicked, this, &RqtRscp::publish_request);
	connect(
	    stage_,
	    qOverload<int>(&QSpinBox::valueChanged),
	    this,
	    &RqtRscp::update_stage_name
	);

	publisher_ = node_->create_publisher<UInt8MultiArray>(
	    "rscp/serial/rx", rclcpp::SensorDataQoS()
	);
	update_stage_name(stage_->value());
	update_visible_fields();
	context.addWidget(widget_);
}

void RqtRscp::shutdownPlugin() {
	publisher_.reset();
}

void RqtRscp::saveSettings(
    qt_gui_cpp::Settings &,
    qt_gui_cpp::Settings &instance_settings
) const {
	instance_settings.setValue("request_type", request_type_->currentIndex());
	instance_settings.setValue("arm", arm_->isChecked());
	instance_settings.setValue("stage", stage_->value());
	instance_settings.setValue("latitude", latitude_->value());
	instance_settings.setValue("longitude", longitude_->value());
	instance_settings.setValue("altitude", altitude_->value());
	instance_settings.setValue("radius", radius_->value());
}

void RqtRscp::restoreSettings(
    const qt_gui_cpp::Settings &,
    const qt_gui_cpp::Settings &instance_settings
) {
	request_type_->setCurrentIndex(instance_settings.value("request_type", 0).toInt());
	arm_->setChecked(instance_settings.value("arm", false).toBool());
	stage_->setValue(instance_settings.value("stage", 0).toInt());
	latitude_->setValue(instance_settings.value("latitude", 0.0).toDouble());
	longitude_->setValue(instance_settings.value("longitude", 0.0).toDouble());
	altitude_->setValue(instance_settings.value("altitude", 0.0).toDouble());
	radius_->setValue(instance_settings.value("radius", 0.0).toDouble());
	update_visible_fields();
}

void RqtRscp::update_visible_fields() {
	const auto type = request_type_->currentIndex();
	arm_row_->setVisible(type == 0);
	stage_row_->setVisible(type == 1);
	latitude_row_->setVisible(type == 2 || type == 3);
	longitude_row_->setVisible(type == 2 || type == 3);
	altitude_row_->setVisible(type == 2 || type == 3);
	radius_row_->setVisible(type == 3);
}

void RqtRscp::update_stage_name(int stage) {
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

void RqtRscp::publish_request() {
	rscp::RequestEnvelope request;
	switch (request_type_->currentIndex()) {
	case 0:
		request.mutable_arm_disarm()->set_value(arm_->isChecked());
		break;
	case 1:
		request.mutable_set_stage()->set_value(static_cast<uint32_t>(stage_->value()));
		break;
	case 2: {
		auto *coordinate = request.mutable_navigate_to_gps()->mutable_coordinate();
		coordinate->set_latitude(latitude_->value());
		coordinate->set_longitude(longitude_->value());
		coordinate->set_altitude(static_cast<float>(altitude_->value()));
		break;
	}
	case 3: {
		auto *search = request.mutable_search_area();
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

	std::vector<uint8_t> serialized(request.ByteSizeLong());
	if (!request.SerializeToArray(serialized.data(), serialized.size())) {
		status_->setText("Failed to serialize protobuf");
		return;
	}

	auto framed = cobs_encode(serialized.data(), serialized.size());
	framed.push_back(0);

	UInt8MultiArray message;
	message.data = framed;
	publisher_->publish(message);

	status_->setText(
	    QStringLiteral("Published %1 framed bytes").arg(framed.size())
	);
	bytes_->setText(bytes_to_hex(framed));
}

} // namespace kalman_arc

PLUGINLIB_EXPORT_CLASS(kalman_arc::RqtRscp, rqt_gui_cpp::Plugin)
