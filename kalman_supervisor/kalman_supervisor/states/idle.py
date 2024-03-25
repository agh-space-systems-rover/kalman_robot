from typing import TYPE_CHECKING
from .state import State

if TYPE_CHECKING:
    from src.main import Supervisor

import time
from core import Ueuos


class Idle(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        supervisor.core.ueuos.color = Ueuos.Color.BLUE
        supervisor.core.move_base.clear_costmap()

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        # If the robot is in autonomous mode and a goal is set,
        # send the goal to the move_base and switch to travel.
        if (
            supervisor.core.control.is_autonomous()
            and supervisor.core.control.is_goal_set()
        ):
            supervisor.core.ueuos.color = Ueuos.Color.RED

            time.sleep(3)

            goal, frame = supervisor.core.control.get_goal()

            if supervisor.core.control.has_multiple_waypoints():
                goal = supervisor.waypoints_server.waypoints

            supervisor.core.long_goal_manager.set_goal(
                position=supervisor.core.position,
                move_base=supervisor.core.move_base,
                transfomer=supervisor.core.transformer,
                goal=goal,
                frame_id=frame,
            )

            supervisor.core.long_goal_manager.drive()
            supervisor.to_travel()

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass