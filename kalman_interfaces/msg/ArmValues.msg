# any kind of values for the arm
# like angle or angular velocity

std_msgs/Header header

float32[6] joints # rad or rad/s
float32 jaw # rad or rad/s

# expected movement when values are positive:
# - joints[0]: turns the entire arm clockwise (to the right) [left stick to the right]
# - joints[1]: bends the first joint forwards (used when reaching out with the arm) [left stick up]
# - joints[2]: bends the second joint forwards (used when folding the arm back) [right stick up]
# - joints[3]: rotates the third joint clockwise looking from behind the robot [right stick to the right]
# - joints[4]: pitches the 2-DoF gearbox forwards (end effector looks down) [dpad up]
# - joints[5]: rolls the 2-DoF gearbox clockwise (end effector's upper side tilts to the right) [right trigger]
# - jaw: opens the gripper, the upper jaw rotates upwards [Y button]
# this aligns with the polarity of URDF joints
