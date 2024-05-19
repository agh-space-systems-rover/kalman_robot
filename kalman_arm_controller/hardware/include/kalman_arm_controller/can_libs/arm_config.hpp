#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP

#define POS_VEL_SCALING 0.27f       // reduces velocity while positioning
extern float posScalingFactiors[6]; // compensates for not equal angular velocities

#include "can_types.hpp"
#include "can_vars.hpp"

namespace arm_config
{
    int load_default_config();
} // namespace arm_config

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__ARM_CONFIG_HPP
