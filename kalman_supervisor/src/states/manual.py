from typing import TYPE_CHECKING
from .state import State

if TYPE_CHECKING:
    from ..main import Supervisor

from core import Core


class Manual(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        supervisor.core.move_base.cancel_goal()
        supervisor.core = Core()

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        # Do nothing and immediately switch to idle.
        supervisor.to_idle()

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass