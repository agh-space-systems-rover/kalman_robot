#!/usr/bin/env python3

from kalman_groundstation.core import Server
from kalman_groundstation.modules.platform import platform_ws
from kalman_groundstation.modules.arm import arm_ws
from kalman_groundstation.modules.autonomy import autonomy_ws
from kalman_groundstation.modules.science import science_ws
# from kalman_groundstation.modules.access_point import access_point_ws
from kalman_groundstation.modules.ble_beacon import ble_beacon_ws
import rclpy
import threading


#  __    __     _                    _        _
# / / /\ \ \___| |__  ___  ___   ___| | _____| |_
# \ \/  \/ / _ \ '_ \/ __|/ _ \ / __| |/ / _ \ __|
#  \  /\  /  __/ |_) \__ \ (_) | (__|   <  __/ |_
#   \/  \/ \___|_.__/|___/\___/ \___|_|\_\___|\__|

"""
Initializes the websocket server and includes all the modules.
"""

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('websocket_node')

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    server = Server(node)
    server.include(platform_ws.server_node)
    server.include(arm_ws.server_node)
    server.include(autonomy_ws.server_node)
    server.include(science_ws.server_node)
    # server.include(access_point_ws.server_node)
    server.include(ble_beacon_ws.server_node)

    server.run()


if __name__ == "__main__":
    main()
