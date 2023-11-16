import math
from typing import TYPE_CHECKING, Tuple

import numpy as np

from core.mode import Mode
from .state import State

if TYPE_CHECKING:
    from ..main import Supervisor

Vec2 = Tuple[float, float]

# Locate mode is used only for finding the second tag after the first one was approached.
class Locate(State):
    # Number of revolutions of the spiral per 1 progress unit.
    SPIRAL_REVOLUTIONS = 3
    # What should be the distance between the revolutions of the spiral
    SPIRAL_REVOLUTION_WIDTH = 2
    # Thus the maximum radius of the spiral is REVOLUTIONS * REVOLUTION_WIDTH
    # By how much is progress increased each time a goal on the spiral is reached?
    PROGRESS_INCREMENT = 0.02

    wander_progress = 0
    wander_origin: Vec2 = (0, 0)

    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        Locate.wander_progress = 0
        Locate.wander_origin = supervisor.core.position.odom
        pass

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        # If both tags were found, drive through the gate.
        mode = supervisor.core.control.get_mode()
        if mode == Mode.TAG_GATE and all(supervisor.core.tag_detector.tags):
            supervisor.to_cross()
            return

        distance_to_goal = supervisor.core.move_base.distance_to_goal(
            supervisor.core.position.odom
        )
        goal_reached = supervisor.core.move_base.is_goal_reached()

        # Set a new goal when the previous one is due or if the spiral hasn't started yet.
        if Locate.wander_progress == 0 or distance_to_goal < 2 or goal_reached:
            supervisor.core.debug_publisher.publish(
                "Exploring around the spiral. Progress: {}%".format(
                    int(Locate.wander_progress * 100)
                )
            )
            # Use sqrt to make the spiral less dense in the beginning and more uniform overall.
            spiral_offset = Locate.spiral(math.sqrt(Locate.wander_progress))
            Locate.wander_progress += Locate.PROGRESS_INCREMENT
            spiral_odom = np.array(Locate.wander_origin) + np.array(spiral_offset)
            spiral_odom = (spiral_odom[0], spiral_odom[1])
            supervisor.core.move_base.send_goal(frame_id="odom", position=spiral_odom)

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass

    # Parametric spiral curve
    # - progress is from 0 to 1
    @staticmethod
    def spiral(progress: float) -> Vec2:
        t = 2 * np.pi * Locate.SPIRAL_REVOLUTIONS * progress
        r = Locate.SPIRAL_REVOLUTIONS * Locate.SPIRAL_REVOLUTION_WIDTH * progress

        return (r * np.cos(t), r * np.sin(t))


# A test of the spiral:
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    for p in np.linspace(0, 1, int(1 / Locate.PROGRESS_INCREMENT)):
        x, y = Locate.spiral(math.sqrt(p))
        plt.scatter(x, y)

    plt.show()