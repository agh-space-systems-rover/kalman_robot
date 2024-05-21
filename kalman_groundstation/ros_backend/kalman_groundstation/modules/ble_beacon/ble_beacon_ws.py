from kalman_groundstation.core import ServerNode
from std_msgs.msg import Float32

server_node = ServerNode()
server_node.publish_to_websocket("/kalman_rover/ble_beacon_signal", Float32)
