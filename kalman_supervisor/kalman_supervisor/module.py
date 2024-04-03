from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from kalman_supervisor.supervisor_node import Supervisor


class Module:
    def __init__(self, name: str) -> None:
        self.name = name

        self.supervisor: Supervisor  # just a type hint
        # Supervisor is set after construction.

    def configure(self) -> None:
        pass

    def activate(self) -> None:
        pass

    def tick(self) -> None:
        pass

    def deactivate(self) -> None:
        pass
