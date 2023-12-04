from core import Core
from kalman_interfaces.msg import SupervisorStatus
from transitions.extensions import GraphMachine
from states import State, TRANSITIONS, STATE_TO_INT
from waypoints_server import WaypointsServer


class Supervisor:
    def __init__(self):
        for state in State:
            setattr(self.__class__, f"on_enter_{state.name}", state.value.on_enter)
            setattr(self.__class__, f"on_exit_{state.name}", state.value.on_exit)

        self.waypoints_server = WaypointsServer()
        self.core = Core()
        self.machine = GraphMachine(
            model=self, states=State, transitions=TRANSITIONS, initial=State.MANUAL
        )

    def run(self):
        self.core.debug_publisher.publish(f"State: {self.state.name}")
        self.core.debug_publisher.publish(f"Tags: {self.core.tag_detector.tags}")
        self.core.status_publisher.publish(
            SupervisorStatus(
                status=STATE_TO_INT[self.state.name],
                tag1_detected=bool(self.core.tag_detector.tags[0]),
                tag2_detected=bool(self.core.tag_detector.tags[1]),
            )
        )

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