#!/usr/bin/env python3

from kalman_groundstation.modules.arm import arm_bridge
from kalman_groundstation.modules.platform import platform_bridge
from kalman_groundstation.modules.autonomy import autonomy_bridge
# from modules.science import science_service, science_bridge
# from modules.access_point import access_point_bridge
import rclpy


#  __                 _
# / _\ ___ _ ____   _(_) ___ ___  ___
# \ \ / _ \ '__\ \ / / |/ __/ _ \/ __|
# _\ \  __/ |   \ V /| | (_|  __/\__ \
# \__/\___|_|    \_/ |_|\___\___||___/


BRIDGES = [
    arm_bridge.ArmBridge,
    platform_bridge.PlatformBridge,
    autonomy_bridge.AutonomyBridge,
    # science_service.Science,
    # science_bridge.ScienceBridge,
    # access_point_bridge.AccessPointBridge,
]


# if __name__ == "__main__":
#     rospy.init_node("ground_station_bridge")

#     instances = []
#     for Bridge in BRIDGES:
#         instances.append(Bridge())

#     rospy.spin()
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ground_station_bridge')
    instances = []
    for Bridge in BRIDGES:
        instances.append(Bridge(node))

    rclpy.spin(node)


if __name__ == "__main__":
    main()
