#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP

#include "kalman_arm_controller/can_libs/can_types.hpp"

namespace arm_config {
constexpr armConfig_t load_default_config() {
	armConfig_t arm_config{};
	arm_config.jointNumber                 = 6;
	arm_config.jointCommandRefreshTime_ms  = 50;
	arm_config.jointCommunicationTimeout   = 500;
	arm_config.canRoverStatusSendPeriod_ms = 50;

	arm_config.joint[1] = {
	    .maxVelocity_deg_s          = 45.0F,
	    .maxAcceleration_deg_ss     = 15.0F,
	    .maxTorque_Nm               = 61.0F,
	    .minPosition_deg            = -200.0F,
	    .maxPosition_deg            = 200.0F,
	    .positionAfterPositioning   = 0.0F,
	    .idleTorque_Nm              = 30.0F,
	    .defVelocity_deg_s          = 5.0F,
	    .defAcceleration_deg_ss     = 100.0F,
	    .defAcceleration_pos_deg_ss = 1000.0F,
	    .gearRatio                  = 0.0125F,
	    .invertDirection            = 0,
	    .requirePositioning         = 0,
	    .positioningOrder           = 3,
	    .positioningVelocity        = 15.0F,
	    .positioningTimeout         = 20000,
	    .differential               = 0,
	};

	arm_config.joint[2] = {
	    .maxVelocity_deg_s          = 45,
	    .maxAcceleration_deg_ss     = 15,
	    .maxTorque_Nm               = 101,
	    .minPosition_deg            = -120,
	    .maxPosition_deg            = 120,
	    .positionAfterPositioning   = 0,
	    .idleTorque_Nm              = 101,
	    .defVelocity_deg_s          = 15,
	    .defAcceleration_deg_ss     = 100,
	    .defAcceleration_pos_deg_ss = 1000,
	    .gearRatio                  = 0.00625F,
	    .invertDirection            = 0,
	    .requirePositioning         = 0,
	    .positioningOrder           = 2,
	    .positioningVelocity        = 15,
	    .positioningTimeout         = 20000,
	    .differential               = 0,
	};

	arm_config.joint[3] = {
	    .maxVelocity_deg_s          = 45,
	    .maxAcceleration_deg_ss     = 15,
	    .maxTorque_Nm               = 101,
	    .minPosition_deg            = -180,
	    .maxPosition_deg            = 180,
	    .positionAfterPositioning   = 0,
	    .idleTorque_Nm              = 101,
	    .defVelocity_deg_s          = 15,
	    .defAcceleration_deg_ss     = 100,
	    .defAcceleration_pos_deg_ss = 1000,
	    .gearRatio                  = 0.00625F,
	    .invertDirection            = 1,
	    .requirePositioning         = 0,
	    .positioningOrder           = 4,
	    .positioningVelocity        = 15,
	    .positioningTimeout         = 20000,
	    .differential               = 0,
	};

	arm_config.joint[4] = {
	    .maxVelocity_deg_s          = 45,
	    .maxAcceleration_deg_ss     = 15,
	    .maxTorque_Nm               = 61,
	    .minPosition_deg            = -360,
	    .maxPosition_deg            = 360,
	    .positionAfterPositioning   = 1.6F,
	    .idleTorque_Nm              = 61,
	    .defVelocity_deg_s          = 5,
	    .defAcceleration_deg_ss     = 100,
	    .defAcceleration_pos_deg_ss = 1000,
	    .gearRatio                  = 0.01F,
	    .invertDirection            = 0,
	    .requirePositioning         = 0,
	    .positioningOrder           = 4,
	    .positioningVelocity        = -15,
	    .positioningTimeout         = 20000,
	    .differential               = 0,
	};

	arm_config.joint[5] = {
	    .maxVelocity_deg_s          = 360,
	    .maxAcceleration_deg_ss     = 360,
	    .maxTorque_Nm               = 100,
	    .minPosition_deg            = -100,
	    .maxPosition_deg            = 100,
	    .positionAfterPositioning   = 0,
	    .idleTorque_Nm              = 100,
	    .defVelocity_deg_s          = 5,
	    .defAcceleration_deg_ss     = 100,
	    .defAcceleration_pos_deg_ss = 1000,
	    .gearRatio                  = static_cast<float>(38.0 / 58.0 / 50.0),
	    .invertDirection            = 0,
	    .requirePositioning         = 0,
	    .positioningOrder           = 1,
	    .positioningVelocity        = -15,
	    .positioningTimeout         = 2000,
	    .differential               = 6,
	};

	arm_config.joint[6] = {
	    .maxVelocity_deg_s          = 360,
	    .maxAcceleration_deg_ss     = 360,
	    .maxTorque_Nm               = 60,
	    .minPosition_deg            = -36000,
	    .maxPosition_deg            = 36000,
	    .positionAfterPositioning   = 0,
	    .idleTorque_Nm              = 60,
	    .defVelocity_deg_s          = 5,
	    .defAcceleration_deg_ss     = 100,
	    .defAcceleration_pos_deg_ss = 1000,
	    .gearRatio                  = static_cast<float>(38.0 / 58.0 / 50.0),
	    .invertDirection            = 0,
	    .requirePositioning         = 0,
	    .positioningOrder           = 1,
	    .positioningVelocity        = -15,
	    .positioningTimeout         = 2000,
	    .differential               = 5,
	};

	return arm_config;
}
} // namespace arm_config

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP
