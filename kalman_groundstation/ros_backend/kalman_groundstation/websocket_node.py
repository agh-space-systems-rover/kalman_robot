#!/usr/bin/env python3

from core import Server
from modules.platform import platform_ws
from modules.arm import arm_ws
from modules.autonomy import autonomy_ws
from modules.science import science_ws
from modules.access_point import access_point_ws
from modules.ble_beacon import ble_beacon_ws
import rclpy


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
    server = Server(node)
    server.include(platform_ws.server_node)
    server.include(arm_ws.server_node)
    server.include(autonomy_ws.server_node)
    server.include(science_ws.server_node)
    server.include(access_point_ws.server_node)
    server.include(ble_beacon_ws.server_node)

    server.run()


if __name__ == "__main__":
    main()
