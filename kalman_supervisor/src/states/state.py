from typing import TYPE_CHECKING
from abc import ABC, abstractmethod

if TYPE_CHECKING:
    from ..main import Supervisor

class State(ABC):
    @staticmethod
    @abstractmethod
    def on_enter(supervisor: "Supervisor") -> None:
        ...

    @staticmethod
    @abstractmethod
    def run(supervisor: "Supervisor") -> None:
        ...

    @staticmethod
    @abstractmethod
    def on_exit(supervisor: "Supervisor") -> None:
        ...