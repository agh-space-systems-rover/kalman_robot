from typing import Tuple, Optional, Union, List
import numpy as np

from .nav2_navigator import MoveBase
from .position import Position
from .transformer import Transformer


Vec2 = Tuple[float, float]


class LongGoalManager:
    # Limited by global costmap size
    GOAL_LEN = 45.0

    ACCEPTABLE_DISTANCE = 0.5

    def __init__(self) -> None:
        self.__position: Optional[Position] = None
        self.__move_base: Optional[MoveBase] = None
        self.__goal: Optional[Vec2] = None
        self.__first_call = True

    def set_goal(
        self,
        position: Position,
        move_base: MoveBase,
        transfomer: Transformer,
        goal: Union[Vec2, List[Vec2]],
        frame_id: str,
    ):
        self.__position = position
        self.__move_base = move_base

        if isinstance(goal, tuple):  # Vec2
            self.__goals_stack = [transfomer.transform2D(goal, frame_id, "odom")]
        else:  # List[Vec2]
            self.__goals_stack = [
                transfomer.transform2D(x, frame_id, "odom") for x in goal
            ][::-1]

        self.__goal = self.__goals_stack.pop()

    # Should be used as a loop condition
    def drive(self) -> bool:
        goal_reached = (self.__move_base.is_goal_reached() or (
            self.__move_base.distance_to_goal(self.__position.odom)
            < self.ACCEPTABLE_DISTANCE
        )) and not self.__first_call and self.__move_base.distance_to_goal(self.__position.odom) < 6.0
        self.__first_call = False

        if goal_reached and len(self.__goals_stack) == 0:
            return True
        elif goal_reached:
            self.__goal = self.__goals_stack.pop()

        to_goal = np.array(self.__goal) - np.array(self.__position.odom)

        norm = np.linalg.norm(to_goal)
        if norm > self.GOAL_LEN:
            to_goal /= norm
            to_goal *= self.GOAL_LEN

        self.__move_base.send_goal(
            'odom', np.array(self.__position.odom) + to_goal
        )

        return False