from kalman_groundstation.core import ServerNode
from kalman_interfaces.msg import ScienceState, SmartProbe, DrillCommand
from std_msgs.msg import Float32

server_node = ServerNode()
server_node.publish_to_websocket("/station/science/state", ScienceState)
server_node.publish_to_websocket('/station/science/weight', Float32)
server_node.publish_to_websocket("/station/science/smart_probe", SmartProbe)
server_node.publish_from_websocket("/station/science/drill", DrillCommand)
