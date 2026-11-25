#include <gtest/gtest.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <limits>
#include <thread>
#include <utility>

#include "kalman_arm_controller/arm_hardware.hpp"
#include "kalman_arm_controller/can_libs/can_vars.hpp"

using hardware_interface::return_type;

class ArmSimTest : public ::testing::TestWithParam<size_t> {
public:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
	static hardware_interface::HardwareInfo make_info(size_t joint_count) {
		hardware_interface::HardwareInfo info;
		for (size_t i = 0; i < joint_count; ++i) {
			hardware_interface::ComponentInfo joint;
			joint.name = i == 6 ? "arm_joint_jaw" : "arm_joint_" + std::to_string(i + 1);
			for (const std::string name : {"position", "velocity"}) {
				hardware_interface::InterfaceInfo interface;
				interface.name = name;
				joint.state_interfaces.push_back(interface);
				joint.command_interfaces.push_back(interface);
			}
			info.joints.push_back(joint);
		}
		return info;
	}

	void SetUp() override {
		system = loader.createSharedInstance("kalman_arm_controller/ArmSimSystem");
		ASSERT_EQ(system->on_init(make_info(GetParam())), kalman_arm_controller::CallbackReturn::SUCCESS);
		commands = system->export_command_interfaces();
		states = system->export_state_interfaces();
		ASSERT_EQ(commands.size(), 2 * GetParam());
		ASSERT_EQ(states.size(), 2 * GetParam());
	}

	pluginlib::ClassLoader<hardware_interface::SystemInterface> loader{
	    "hardware_interface", "hardware_interface::SystemInterface"};
	std::shared_ptr<hardware_interface::SystemInterface> system;
	std::vector<hardware_interface::CommandInterface> commands;
	std::vector<hardware_interface::StateInterface> states;
	const rclcpp::Time now{0};
	const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);
};

TEST_P(ArmSimTest, VelocityModeIntegratesAllJointsWithoutChangingCanCommands) {
	{
		std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
		CAN_vars::joints.jointCmd[0].moveSetpoint.position_deg = 123.0f;
	}
	for (size_t i = 0; i < GetParam(); ++i) {
		commands[i].set_value(10.0); // Stale position commands must be ignored.
		commands[i + GetParam()].set_value(static_cast<double>(i + 1));
	}
	ASSERT_EQ(system->read(now, period), return_type::OK);
	ASSERT_EQ(system->write(now, period), return_type::OK);
	ASSERT_EQ(system->write(now, period), return_type::OK);
	for (size_t i = 0; i < GetParam(); ++i) {
		EXPECT_DOUBLE_EQ(states[i].get_value(), 0.2 * static_cast<double>(i + 1));
		EXPECT_DOUBLE_EQ(states[i + GetParam()].get_value(), static_cast<double>(i + 1));
	}
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
	EXPECT_FLOAT_EQ(CAN_vars::joints.jointCmd[0].moveSetpoint.position_deg, 123.0f);
}

TEST_P(ArmSimTest, PositionModeFollowsCommandsAndIgnoresInvalidMode) {
	auto node = rclcpp::Node::make_shared("arm_sim_test");
	auto publisher = node->create_publisher<std_msgs::msg::UInt8>("/change_control_type", 10);
	for (size_t i = 0; i < GetParam(); ++i) {
		commands[i].set_value(0.2 * static_cast<double>(i + 1));
	}
	std_msgs::msg::UInt8 mode;
	mode.data = 0;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
	do {
		publisher->publish(mode);
		ASSERT_EQ(system->read(now, period), return_type::OK);
		ASSERT_EQ(system->write(now, period), return_type::OK);
		if (states[0].get_value() == 0.2) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	} while (std::chrono::steady_clock::now() < deadline);
	for (size_t i = 0; i < GetParam(); ++i) {
		EXPECT_DOUBLE_EQ(states[i].get_value(), commands[i].get_value());
	}
	mode.data = 255;
	publisher->publish(mode);
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	ASSERT_EQ(system->read(now, period), return_type::OK);
	commands[0].set_value(0.3);
	ASSERT_EQ(system->write(now, period), return_type::OK);
	EXPECT_DOUBLE_EQ(states[0].get_value(), 0.3);
}

TEST_P(ArmSimTest, InvalidVelocityOrPeriodDoesNotPartiallyUpdateState) {
	commands[GetParam()].set_value(1.0);
	commands[2 * GetParam() - 1].set_value(std::numeric_limits<double>::quiet_NaN());
	EXPECT_EQ(system->write(now, period), return_type::ERROR);
	EXPECT_DOUBLE_EQ(states[0].get_value(), 0.0);
	commands[2 * GetParam() - 1].set_value(0.0);
	EXPECT_EQ(system->write(now, rclcpp::Duration::from_seconds(-0.1)), return_type::ERROR);
	EXPECT_DOUBLE_EQ(states[0].get_value(), 0.0);
}

TEST_P(ArmSimTest, ExportsNamedZeroInitializedInterfacesForEveryJoint) {
	const auto info = make_info(GetParam());
	for (size_t i = 0; i < GetParam(); ++i) {
		for (size_t offset : {size_t{0}, GetParam()}) {
			const std::string interface_name = offset == 0 ? "position" : "velocity";
			const std::string name = info.joints[i].name + "/" + interface_name;
			EXPECT_EQ(commands[i + offset].get_name(), name);
			EXPECT_EQ(states[i + offset].get_name(), name);
			EXPECT_DOUBLE_EQ(commands[i + offset].get_value(), 0.0);
			EXPECT_DOUBLE_EQ(states[i + offset].get_value(), 0.0);
		}
	}
}

TEST_P(ArmSimTest, RejectsInvalidJointLayouts) {
	std::vector<hardware_interface::HardwareInfo> invalid_infos;
	for (size_t count : {size_t{0}, size_t{5}, size_t{8}}) {
		invalid_infos.push_back(make_info(count));
	}
	for (size_t i = 0; i < 6; ++i) {
		auto info = make_info(GetParam());
		info.joints[i].name = "unexpected_joint";
		invalid_infos.push_back(info);
	}
	auto swapped = make_info(GetParam());
	std::swap(swapped.joints[0], swapped.joints[1]);
	invalid_infos.push_back(swapped);
	auto wrong_jaw = make_info(7);
	wrong_jaw.joints[6].name = "arm_joint_7";
	invalid_infos.push_back(wrong_jaw);
	auto misplaced_jaw = make_info(7);
	std::swap(misplaced_jaw.joints[5], misplaced_jaw.joints[6]);
	invalid_infos.push_back(misplaced_jaw);

	for (size_t i = 0; i < invalid_infos.size(); ++i) {
		SCOPED_TRACE(i);
		auto sim = loader.createSharedInstance("kalman_arm_controller/ArmSimSystem");
		EXPECT_EQ(sim->on_init(invalid_infos[i]), kalman_arm_controller::CallbackReturn::ERROR);
		EXPECT_TRUE(sim->export_state_interfaces().empty());
		EXPECT_TRUE(sim->export_command_interfaces().empty());
		kalman_arm_controller::ArmSystem hardware;
		EXPECT_EQ(hardware.on_init(invalid_infos[i]), kalman_arm_controller::CallbackReturn::ERROR);
		EXPECT_TRUE(hardware.export_state_interfaces().empty());
		EXPECT_TRUE(hardware.export_command_interfaces().empty());
	}
}

TEST_P(ArmSimTest, PluginCanBeDestroyedWhileRosIsRunning) {
	commands.clear();
	states.clear();
	system.reset();
	EXPECT_TRUE(rclcpp::ok());
}

INSTANTIATE_TEST_SUITE_P(JointCounts, ArmSimTest, ::testing::Values(size_t{6}, size_t{7}));
