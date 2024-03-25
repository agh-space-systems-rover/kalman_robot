import rclpy
from rclpy.node import Node
from transitions.extensions import GraphMachine

from core import Core
from states import State, TRANSITIONS

class Supervisor(Node):
    def __init__(self):
        for state in State:
            # Create self.on_enter_{state} and self.on_exit_{state} methods
            setattr(self.__class__, f"on_enter_{state.name}", state.value.on_enter)
            setattr(self.__class__, f"on_exit_{state.name}", state.value.on_exit)

        self.core = Core()
        self.machine = GraphMachine(
            model=self, states=State, transitions=TRANSITIONS, initial=State.MANUAL
        )
        self.create_timer(0.1, self.run)

    def run(self):
        # Switch to manual mode if the robot is not in autonomous mode.
        # Do not switch if the robot is idling.
        if (self.core.control.is_autonomous() == False) and self.state not in (
            State.MANUAL,
            State.IDLE,
        ):
            self.core.debug_publisher.publish("Switching to MANUAL")
            self.to_manual()
        else:
            self.state.value.run(self)

def main():
    try:
        rclpy.init()
        node = Supervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
