import math
from typing import TYPE_CHECKING

import numpy as np

from core.mode import Mode
from .state import State
from typing import Tuple

if TYPE_CHECKING:
    from src.main import Supervisor

Vec2 = Tuple[float, float]

# Explore mode searches for tags and either approaches one of them or
# crosses the gate if both tags were found.
class Explore(State):
    # Number of revolutions of the spiral per 1 progress unit.
    SPIRAL_REVOLUTIONS = 3
    # What should be the distance between the revolutions of the spiral
    SPIRAL_REVOLUTION_WIDTH = 4
    # Thus the maximum radius of the spiral is REVOLUTIONS * REVOLUTION_WIDTH
    # By how much is progress increased each time a goal on the spiral is reached?
    PROGRESS_INCREMENT = 0.02

    wander_progress = 0
    wander_origin: Vec2 = (0, 0)

    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        Explore.wander_progress = 0
        Explore.wander_origin = supervisor.core.position.odom
        pass

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        # Approach any points of interest if they were detected.

        tag1, tag2 = supervisor.core.tag_detector.tags
        if any((tag1, tag2)):
            # Only one tag was found. Start approach.
            if (
                supervisor.core.control.get_mode() == Mode.TAG_GOAL
                or tag1 is None
                or tag2 is None
            ):
                found_tag = tag1 or tag2
                supervisor.core.move_base.send_goal(
                    found_tag.frame_id, found_tag.position
                )
                supervisor.to_approach()
                return
            # Both tags were found. Drive through the gate.
            elif supervisor.core.control.get_mode() == Mode.TAG_GATE:
                supervisor.to_cross()
                return

        distance_to_goal = supervisor.core.move_base.distance_to_goal(
            supervisor.core.position.odom
        )
        goal_reached = supervisor.core.move_base.is_goal_reached()

        # Set a new goal when the previous one is due or if the spiral hasn't started yet.
        if Explore.wander_progress == 0 or distance_to_goal < 2 or goal_reached:
            supervisor.core.debug_publisher.publish(
                "Exploring around the spiral. Progress: {}%".format(
                    int(Explore.wander_progress * 100)
                )
            )
            # Use sqrt to make the spiral less dense in the beginning and more uniform overall.
            spiral_offset = Explore.spiral(math.sqrt(Explore.wander_progress))
            Explore.wander_progress += Explore.PROGRESS_INCREMENT
            spiral_odom = np.array(Explore.wander_origin) + np.array(spiral_offset)
            spiral_odom = (spiral_odom[0], spiral_odom[1])
            supervisor.core.move_base.send_goal(frame_id="odom", position=spiral_odom)

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass

    # Parametric spiral curve
    # - progress is from 0 to 1
    @staticmethod
    def spiral(progress: float) -> Vec2:
        t = 2 * np.pi * Explore.SPIRAL_REVOLUTIONS * progress
        r = Explore.SPIRAL_REVOLUTIONS * Explore.SPIRAL_REVOLUTION_WIDTH * progress

        return (r * np.cos(t), r * np.sin(t))