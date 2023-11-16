from typing import TYPE_CHECKING
from .state import State
import time

if TYPE_CHECKING:
    from ..main import Supervisor

from core import Ueuos

class Completed(State):
    @staticmethod
    def on_enter(supervisor: "Supervisor") -> None:
        # supervisor.core.move_base.cancel_goal()
        time.sleep(2)
        supervisor.core.ueuos.color = Ueuos.Color.GREEN
        supervisor.core.control.completed()

    @staticmethod
    def run(supervisor: "Supervisor") -> None:
        pass

    @staticmethod
    def on_exit(supervisor: "Supervisor") -> None:
        pass