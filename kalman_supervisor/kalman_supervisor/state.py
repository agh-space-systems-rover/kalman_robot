from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from kalman_supervisor.supervisor_node import Supervisor

class State:
    def __init__(self, name: str) -> None:
        self.name = name

        self.supervisor: Supervisor # just a type hint
        # Supervisor is set later in Supervisor.tick() after a transition.

    def enter(self) -> None:
        pass

    def tick(self) -> str | None:
        pass

    def exit(self) -> None:
        pass
