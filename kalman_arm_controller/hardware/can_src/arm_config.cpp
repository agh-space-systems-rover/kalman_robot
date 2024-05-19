#include "kalman_arm_controller/can_libs/arm_config.hpp"

namespace arm_config
{
int load_default_config()
{
  CAN_vars::arm_config.jointNumber = 6;
  CAN_vars::arm_config.jointCommandRefreshTime_ms = 50;
  CAN_vars::arm_config.jointCommunicationTimeout = 500;
  CAN_vars::arm_config.canRoverStatusSendPeriod_ms = 50;

  CAN_vars::arm_config.joint[1].maxVelocity_deg_s = 45;
  CAN_vars::arm_config.joint[1].maxAcceleration_deg_ss = 15;
  CAN_vars::arm_config.joint[1].maxTorque_Nm = 61;
  CAN_vars::arm_config.joint[1].minPosition_deg = -200;
  CAN_vars::arm_config.joint[1].maxPosition_deg = 200;
  CAN_vars::arm_config.joint[1].gearRatio = 0.0125;
  CAN_vars::arm_config.joint[1].invertDirection = 0;
  CAN_vars::arm_config.joint[1].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[1].positioningVelocity = 15;
  CAN_vars::arm_config.joint[1].positioningTimeout = 20000;
  CAN_vars::arm_config.joint[1].positioningOrder = 3;
  CAN_vars::arm_config.joint[1].positionAfterPositioning = 0;
  CAN_vars::arm_config.joint[1].idleTorque_Nm = 30;
  CAN_vars::arm_config.joint[1].defVelocity_deg_s = 5;
  CAN_vars::arm_config.joint[1].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[1].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[1].differential = 0;

  CAN_vars::arm_config.joint[2].maxVelocity_deg_s = 45;
  CAN_vars::arm_config.joint[2].maxAcceleration_deg_ss = 15;
  CAN_vars::arm_config.joint[2].maxTorque_Nm = 101;
  CAN_vars::arm_config.joint[2].minPosition_deg = -120;
  CAN_vars::arm_config.joint[2].maxPosition_deg = 120;
  CAN_vars::arm_config.joint[2].gearRatio = 0.00625;
  CAN_vars::arm_config.joint[2].invertDirection = 0;
  CAN_vars::arm_config.joint[2].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[2].positioningVelocity = 15;
  CAN_vars::arm_config.joint[2].positioningTimeout = 20000;
  CAN_vars::arm_config.joint[2].positioningOrder = 2;
  CAN_vars::arm_config.joint[2].positionAfterPositioning = 0;
  CAN_vars::arm_config.joint[2].idleTorque_Nm = 101;
  CAN_vars::arm_config.joint[2].defVelocity_deg_s = 15;  // bylo 15 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[2].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[2].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[2].differential = 0;

  CAN_vars::arm_config.joint[3].maxVelocity_deg_s = 45;
  CAN_vars::arm_config.joint[3].maxAcceleration_deg_ss = 15;
  CAN_vars::arm_config.joint[3].maxTorque_Nm = 101;
  CAN_vars::arm_config.joint[3].minPosition_deg = -180;
  CAN_vars::arm_config.joint[3].maxPosition_deg = 180;
  CAN_vars::arm_config.joint[3].gearRatio = 0.00625;
  CAN_vars::arm_config.joint[3].invertDirection = 1;
  CAN_vars::arm_config.joint[3].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[3].positioningVelocity = 15;
  CAN_vars::arm_config.joint[3].positioningTimeout = 20000;
  CAN_vars::arm_config.joint[3].positioningOrder = 4;
  CAN_vars::arm_config.joint[3].positionAfterPositioning = 0;
  CAN_vars::arm_config.joint[3].idleTorque_Nm = 101;
  CAN_vars::arm_config.joint[3].defVelocity_deg_s = 15;  // bylo 15 !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[3].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[3].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[3].differential = 0;

  CAN_vars::arm_config.joint[4].maxVelocity_deg_s = 45;
  CAN_vars::arm_config.joint[4].maxAcceleration_deg_ss = 15;
  CAN_vars::arm_config.joint[4].maxTorque_Nm = 61;
  CAN_vars::arm_config.joint[4].minPosition_deg = -360;
  CAN_vars::arm_config.joint[4].maxPosition_deg = 360;
  CAN_vars::arm_config.joint[4].gearRatio = 0.01;
  CAN_vars::arm_config.joint[4].invertDirection = 0;
  CAN_vars::arm_config.joint[4].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[4].positioningVelocity = -15;
  CAN_vars::arm_config.joint[4].positioningTimeout = 20000;
  CAN_vars::arm_config.joint[4].positioningOrder = 4;
  CAN_vars::arm_config.joint[4].positionAfterPositioning = 1.6;
  CAN_vars::arm_config.joint[4].idleTorque_Nm = 61;
  CAN_vars::arm_config.joint[4].defVelocity_deg_s = 5;
  CAN_vars::arm_config.joint[4].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[4].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[4].differential = 0;

  CAN_vars::arm_config.joint[5].maxVelocity_deg_s = 360;
  CAN_vars::arm_config.joint[5].maxAcceleration_deg_ss = 360;
  CAN_vars::arm_config.joint[5].maxTorque_Nm = 100;
  CAN_vars::arm_config.joint[5].minPosition_deg = -100;
  CAN_vars::arm_config.joint[5].maxPosition_deg = 100;
  CAN_vars::arm_config.joint[5].gearRatio = (38.0 / 58.0 / 50.0);
  CAN_vars::arm_config.joint[5].invertDirection = 0;
  CAN_vars::arm_config.joint[5].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[5].positioningVelocity = -15;
  CAN_vars::arm_config.joint[5].positioningTimeout = 2000;
  CAN_vars::arm_config.joint[5].positioningOrder = 1;
  CAN_vars::arm_config.joint[5].positionAfterPositioning = 0;
  CAN_vars::arm_config.joint[5].idleTorque_Nm = 100;
  CAN_vars::arm_config.joint[5].defVelocity_deg_s = 5;
  CAN_vars::arm_config.joint[5].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[5].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[5].differential = 6;

  CAN_vars::arm_config.joint[6].maxVelocity_deg_s = 360;
  CAN_vars::arm_config.joint[6].maxAcceleration_deg_ss = 360;
  CAN_vars::arm_config.joint[6].maxTorque_Nm = 60;
  CAN_vars::arm_config.joint[6].minPosition_deg = -36000;
  CAN_vars::arm_config.joint[6].maxPosition_deg = 36000;
  CAN_vars::arm_config.joint[6].gearRatio = (38.0 / 58.0 / 50.0);
  CAN_vars::arm_config.joint[6].invertDirection = 0;
  CAN_vars::arm_config.joint[6].requirePositioning = 0;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  CAN_vars::arm_config.joint[6].positioningVelocity = -15;
  CAN_vars::arm_config.joint[6].positioningTimeout = 2000;
  CAN_vars::arm_config.joint[6].positioningOrder = 1;
  CAN_vars::arm_config.joint[6].positionAfterPositioning = 0;
  CAN_vars::arm_config.joint[6].idleTorque_Nm = 60;
  CAN_vars::arm_config.joint[6].defVelocity_deg_s = 5;
  CAN_vars::arm_config.joint[6].defAcceleration_deg_ss = 100;
  CAN_vars::arm_config.joint[6].defAcceleration_pos_deg_ss = 1000;
  CAN_vars::arm_config.joint[6].differential = 5;

  return 0;
}

}  // namespace arm_config
