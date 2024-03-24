from core import ServerNode
from kalman_groundstation.msg import ArmFkCommand, ArmIkCommand, ArmState

server_node = ServerNode()
server_node.publish_from_websocket("/station/arm/fk/command", ArmFkCommand)
server_node.publish_from_websocket("/station/arm/ik/command", ArmIkCommand)
server_node.publish_to_websocket("/station/arm/state", ArmState)