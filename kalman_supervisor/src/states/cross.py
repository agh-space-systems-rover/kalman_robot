from typing import TYPE_CHECKING
from .state import State

if TYPE_CHECKING:
    from ..main import Supervisor

from core.gate_manager import GateManager


class Cross(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        pass

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        goal_reached = supervisor.core.move_base.is_goal_reached()
        state = supervisor.core.gate_manager.state
        frame = supervisor.core.tag_detector.tags[0].frame_id

        # Recalculate goals each time for the most accurate positions
        supervisor.core.gate_manager.calculate(supervisor.core.tag_detector.tags)

        if state == GateManager.State.LOOKING_FOR_GATE:
            supervisor.core.move_base.send_goal(
                frame, supervisor.core.gate_manager.front
            )
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.GO_FAR_FRONT and goal_reached:
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.POS_FAR_FRONT:
            supervisor.core.move_base.clear_costmap()
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.CM_CLEAR:
            supervisor.core.move_base.send_goal(
                frame, supervisor.core.gate_manager.front_close
            )
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.GO_CLOSE_FRONT and goal_reached:
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.POS_CLOSE_FRONT:
            supervisor.core.move_base.send_goal(
                frame, supervisor.core.gate_manager.back
            )
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.GO_BACK and goal_reached:
            supervisor.core.gate_manager.next()

        elif state == GateManager.State.CROSSED and goal_reached:
            supervisor.to_completed()

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass