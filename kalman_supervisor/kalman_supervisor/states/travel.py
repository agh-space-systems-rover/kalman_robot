from typing import TYPE_CHECKING
from .state import State

if TYPE_CHECKING:
    from ..main import Supervisor

from core import Mode


class Travel(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        supervisor.core.tag_detector.reset() # Avoid storing old tags

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        mode: Mode = supervisor.core.control.get_mode()

        tag1, tag2 = supervisor.core.tag_detector.tags

        # If some tag was found, start approach or drive through the gate.
        # Otherwise start searching for tags.
        if any((tag1, tag2)):
            # Only one tag was found. Start approach.
            if mode == Mode.TAG_GOAL and (tag1 is None or tag2 is None):
                found_tag = tag1 or tag2
                supervisor.core.move_base.clear_costmap()
                supervisor.core.move_base.send_goal(
                    found_tag.frame_id, found_tag.position
                )
                supervisor.to_approach()
                return
            # Both tags were found. Drive through the gate.
            elif mode == Mode.TAG_GATE:
                supervisor.to_cross()
                return

        goal_reached: bool = supervisor.core.long_goal_manager.drive()

        if goal_reached:

            if mode == Mode.SIMPLE_GOAL:
                # TODO: correcting simple goal?
                supervisor.to_completed()
                return

            # No tags were found. Start searching for tags.
            else:
                supervisor.to_explore()
                return

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass