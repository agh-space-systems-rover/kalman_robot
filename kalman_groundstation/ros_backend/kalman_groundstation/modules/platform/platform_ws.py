from kalman_groundstation.core import ServerNode
from kalman_interfaces.msg import WheelsCommand
from kalman_interfaces.msg import PlatformState
from kalman_interfaces.msg import MotorsTemperature
from std_msgs.msg import Float32MultiArray

server_node = ServerNode()
server_node.publish_from_websocket("/station/wheels/command", WheelsCommand)
server_node.publish_from_websocket("/station/digging/command", Float32MultiArray)
server_node.publish_to_websocket("/station/wheels/state", PlatformState)
server_node.publish_to_websocket("/station/wheels/temperatures", MotorsTemperature)