#include "kalman_arm_controller/can_libs/new_can_driver.hpp"
#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <thread>

namespace {
canfd_frame fast_frame(uint32_t joint, int16_t velocity, int32_t position) {
	canfd_frame frame{};
	frame.can_id = (joint << 7) | CMD_JOINT_FAST_STATUS;
	frame.len = sizeof(jointMotorFastStatus_t);
	const jointMotorFastStatus_t status{velocity, position};
	std::memcpy(frame.data, &status, sizeof(status));
	return frame;
}

class CanFeedbackTest : public ::testing::Test {
protected:
	void SetUp() override {
		std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
		CAN_vars::joints.jointFeedback = {};
		CAN_vars::joints.gripper_position = 0;
	}

	int dispatch(canfd_frame frame) {
		return driver.handle_frame(frame, &CAN_handlers::HANDLES);
	}

	CanDriver driver;
};

TEST_F(CanFeedbackTest, RejectsInvalidJointIdsWithoutChangingFeedback) {
	for (const uint32_t joint : {0U, 7U, 15U, 256U}) {
		auto frame = fast_frame(joint, 123, 456);
		EXPECT_NE(dispatch(frame), 0);
		EXPECT_NE(CAN_handlers::handle_joint_fast_status(
		              frame.can_id, frame.data, frame.len), 0);
		frame.can_id = (joint << 7) | CMD_JOINT_STATUS;
		frame.len = sizeof(jointMotorStatus_t);
		EXPECT_NE(dispatch(frame), 0);
		EXPECT_NE(CAN_handlers::handle_joint_status(
		              frame.can_id, frame.data, frame.len), 0);
	}
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	for (const auto &feedback : CAN_vars::joints.jointFeedback) {
		EXPECT_FALSE(feedback.received);
		EXPECT_EQ(feedback.fastStatus.position, 0);
		EXPECT_FLOAT_EQ(feedback.moveStatus.position_deg, 0.0F);
	}
}

TEST_F(CanFeedbackTest, RejectsWrongPayloadLengthsAndNullData) {
	for (const uint8_t command : std::array<uint8_t, 2>{CMD_JOINT_STATUS, CMD_JOINT_FAST_STATUS}) {
		const auto &handler = CAN_handlers::HANDLES.at(command);
		for (const uint8_t length : {uint8_t{0}, static_cast<uint8_t>(handler.len - 1),
		                             static_cast<uint8_t>(handler.len + 1), uint8_t{65}}) {
			canfd_frame frame{};
			frame.can_id = (1U << 7) | command;
			frame.len = length;
			EXPECT_NE(dispatch(frame), 0);
			EXPECT_NE(handler.func(frame.can_id, frame.data, length), 0);
		}
		EXPECT_NE(handler.func((1U << 7) | command, nullptr, handler.len), 0);
	}
	EXPECT_FALSE(CAN_vars::joints.jointFeedback[0].received);
}

TEST_F(CanFeedbackTest, RejectsFlaggedAndNonStandardIdentifiers) {
	for (const uint32_t flag : {CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG, 0x800U}) {
		auto frame = fast_frame(1, 123, 456);
		frame.can_id |= flag;
		EXPECT_NE(dispatch(frame), 0);
		EXPECT_NE(CAN_handlers::handle_joint_fast_status(
		              frame.can_id, frame.data, frame.len), 0);
	}
	auto frame = fast_frame(1, 123, 456);
	frame.flags = 0x80;
	EXPECT_NE(dispatch(frame), 0);
	EXPECT_NE(driver.handle_frame(frame, nullptr), 0);
	EXPECT_FALSE(CAN_vars::joints.jointFeedback[0].received);
}

TEST_F(CanFeedbackTest, AcceptsValidFdFlagsAndConvertsBeforeReturning) {
	auto frame = fast_frame(3, 200, 10000);
	frame.flags = CANFD_BRS | CANFD_ESI | CANFD_FDF;
	ASSERT_EQ(dispatch(frame), 0);
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	const auto &feedback = CAN_vars::joints.jointFeedback[2];
	EXPECT_TRUE(feedback.received);
	EXPECT_EQ(feedback.fastStatus.position, 10000);
	EXPECT_FLOAT_EQ(feedback.moveStatus.position_deg, -0.625F);
	EXPECT_FLOAT_EQ(feedback.moveStatus.velocity_deg_s, -0.75F);
}

TEST_F(CanFeedbackTest, ConvertsBothDifferentialOutputsWithoutRecursiveLocking) {
	ASSERT_EQ(dispatch(fast_frame(5, 100, 10000)), 0);
	ASSERT_EQ(dispatch(fast_frame(6, 300, 20000)), 0);
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	const auto &first = CAN_vars::joints.jointFeedback[4];
	const auto &second = CAN_vars::joints.jointFeedback[5];
	const float ratio = CAN_vars::arm_config.joint[5].gearRatio;
	EXPECT_TRUE(first.received);
	EXPECT_TRUE(second.received);
	EXPECT_FLOAT_EQ(first.moveStatus.position_deg, 50.0F * ratio);
	EXPECT_FLOAT_EQ(second.moveStatus.position_deg, 150.0F * ratio);
	EXPECT_FLOAT_EQ(first.moveStatus.velocity_deg_s, -60.0F * ratio);
	EXPECT_FLOAT_EQ(second.moveStatus.velocity_deg_s, 120.0F * ratio);
}

TEST_F(CanFeedbackTest, ReaderSeesRawConvertedAndReadinessAsOneTransaction) {
	std::atomic_bool done{false};
	std::atomic_bool coherent{true};
	std::atomic_bool started{false};
	std::thread reader([&] {
		started.store(true);
		do {
			std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
			const auto &feedback = CAN_vars::joints.jointFeedback[0];
			if (feedback.received) {
				const float expected = 0.01F * CAN_vars::arm_config.joint[1].gearRatio *
				                       static_cast<float>(feedback.fastStatus.position);
				if (feedback.moveStatus.position_deg != expected) {
					coherent.store(false);
				}
			}
		} while (!done.load());
	});
	while (!started.load()) {
		std::this_thread::yield();
	}
	for (int32_t position = 1; position <= 2000; ++position) {
		EXPECT_EQ(dispatch(fast_frame(1, 100, position)), 0);
	}
	done.store(true);
	reader.join();
	EXPECT_TRUE(coherent.load());
}

TEST_F(CanFeedbackTest, StandaloneConversionEntryPointsStillAcquireLocks) {
	ASSERT_EQ(dispatch(fast_frame(1, 100, 10000)), 0);
	CAN_vars::update_joint_status();
	CAN_vars::update_single_joint_status(0);
	CAN_vars::calculate_status(0);
	CAN_vars::calculate_status_diff(4, 6);
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_read);
	EXPECT_FLOAT_EQ(CAN_vars::joints.jointFeedback[0].moveStatus.position_deg, 1.25F);
}

TEST_F(CanFeedbackTest, CopiesUnalignedPayloadsAndValidatesDirectHandlers) {
	std::array<uint8_t, sizeof(jointMotorStatus_t) + 1> bytes{};
	bytes[1] = 0x5a;
	ASSERT_EQ(CAN_handlers::handle_joint_status(
	              (6U << 7) | CMD_JOINT_STATUS, bytes.data() + 1,
	              sizeof(jointMotorStatus_t)), 0);
	EXPECT_EQ(std::memcmp(&CAN_vars::joints.jointFeedback[5].status,
	                      bytes.data() + 1, sizeof(jointMotorStatus_t)), 0);
	const jointMotorFastStatus_t fast{100, 10000};
	std::memcpy(bytes.data() + 1, &fast, sizeof(fast));
	ASSERT_EQ(CAN_handlers::handle_joint_fast_status(
	              (1U << 7) | CMD_JOINT_FAST_STATUS, bytes.data() + 1, sizeof(fast)), 0);
	EXPECT_FLOAT_EQ(CAN_vars::joints.jointFeedback[0].moveStatus.position_deg, 1.25F);
	EXPECT_NE(CAN_handlers::handle_joint_fast_status(
	              (1U << 7) | CMD_JOINT_STATUS, bytes.data() + 1, sizeof(fast)), 0);
	const uint16_t gripper = 1234;
	std::memcpy(bytes.data() + 1, &gripper, sizeof(gripper));
	EXPECT_EQ(CAN_handlers::handle_gripper_status(
	              CMD_GET_GRIPPER, bytes.data() + 1, sizeof(gripper)), 0);
	EXPECT_EQ(CAN_vars::joints.gripper_position, gripper);
	EXPECT_NE(CAN_handlers::handle_gripper_status(CMD_GET_GRIPPER, nullptr, 2), 0);
	EXPECT_NE(CAN_handlers::handle_gripper_status(CMD_GET_GRIPPER, bytes.data(), 1), 0);
	EXPECT_NE(CAN_handlers::handle_gripper_status(
	              CMD_GET_GRIPPER | CAN_RTR_FLAG, bytes.data(), 2), 0);
}

TEST_F(CanFeedbackTest, SetpointsSaturateBeforeNarrowing) {
	for (const float sign : {-1.0f, 1.0f}) {
		{
			std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
			auto &command = CAN_vars::joints.jointCmd[0];
			command.moveSetpoint.position_deg = sign * 1.0e30f;
			command.moveSetpoint.velocity_deg_s = sign * 1.0e30f;
		}
		CAN_vars::update_single_joint_setpoint(0);
		const bool negative = (sign < 0.0f) != CAN_vars::arm_config.joint[1].invertDirection;
		std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
		const auto &command = CAN_vars::joints.jointCmd[0];
		EXPECT_EQ(command.setpoint.position_0deg01,
		          negative ? INT32_MIN : INT32_MAX);
		EXPECT_EQ(command.velSetpoint.velocity_0RPM_1,
		          negative ? INT16_MIN : INT16_MAX);
	}
}

TEST_F(CanFeedbackTest, PositionLimitComparisonUsesExactInt32Bound) {
	// 2^31 is the first float above INT32_MAX; it must clamp, not overflow.
	const float wire_position = 2147483648.0f;
	const auto &config = CAN_vars::arm_config.joint[1];
	{
		std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
		auto &command = CAN_vars::joints.jointCmd[0];
		command.moveSetpoint.position_deg = wire_position * config.gearRatio / 100.0f *
		    (config.invertDirection ? -1.0f : 1.0f);
		command.moveSetpoint.velocity_deg_s = 0.0f;
	}
	CAN_vars::update_single_joint_setpoint(0);
	std::lock_guard<std::mutex> lock(CAN_vars::joints.m_write);
	EXPECT_EQ(CAN_vars::joints.jointCmd[0].setpoint.position_0deg01, INT32_MAX);
}

TEST_F(CanFeedbackTest, PropagatesHandlerFailure) {
	const std::unordered_map<uint8_t, canCmdHandler_t> handlers{
	    {CMD_JOINT_FAST_STATUS,
	     {CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t),
	      [](uint32_t, uint8_t *, uint8_t) { return 17; }}}};
	EXPECT_EQ(driver.handle_frame(fast_frame(1, 100, 10000), &handlers), 17);
}
} // namespace
