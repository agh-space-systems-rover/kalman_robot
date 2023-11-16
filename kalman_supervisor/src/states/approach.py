import numpy as np
from typing import TYPE_CHECKING
from .state import State

if TYPE_CHECKING:
    from ..main import Supervisor

from core.mode import Mode

CLEAR_COSTMAP_DISTANCE = 3
DISTANCE_FROM_TAG = 0.75

class Approach(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        pass

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        mode = supervisor.core.control.get_mode()

        if mode == Mode.TAG_GATE and all(supervisor.core.tag_detector.tags):
            supervisor.to_cross()
            return

        if supervisor.core.move_base.is_goal_reached():
            if mode == Mode.TAG_GOAL:
                supervisor.to_completed()
            elif mode == Mode.TAG_GATE:
                supervisor.to_locate()
            return

        # We cannot be sure which tag was found
        tag1, tag2 = supervisor.core.tag_detector.tags
        found_tag = tag1 or tag2

        # Send goal 1 meter from the tag
        rover_pos = np.array(supervisor.core.position.odom)
        tag_pos = np.array(
            supervisor.core.transformer.transform2D(
                found_tag.position, found_tag.frame_id, "odom"
            )
        )
        goal_vec = np.subtract(tag_pos, rover_pos)
        unit_vec = DISTANCE_FROM_TAG * goal_vec / np.linalg.norm(goal_vec)
        final_goal = np.subtract(tag_pos, unit_vec)

        #if np.linalg.norm(goal_vec) < CLEAR_COSTMAP_DISTANCE:
        supervisor.core.move_base.clear_costmap()
        #else:
        supervisor.core.move_base.send_goal("odom", final_goal)

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass