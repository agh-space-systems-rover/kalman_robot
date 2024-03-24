from typing import List, Optional
from kalman_groundstation.core.models.ros_model import RosModel


class ArmJointSpaceGoal(RosModel):
    label: str
    velocity_scaling_factor: Optional[float] = 0.5
    acceleration_scaling_factor: Optional[float] = 0.5


class ArmCartesianSpaceGoal(RosModel):
    label: str
    velocity_scaling_factor: Optional[float] = 0.5
    acceleration_scaling_factor: Optional[float] = 0.5


class ArmGoals(RosModel):
    cartesian_space_goals: List[ArmCartesianSpaceGoal]
    joint_space_goals: List[ArmJointSpaceGoal]

    @classmethod
    def from_dicts(cls, cartesian_dictionary: dict, joint_dictionary: dict):
        cartesian_space_goals = [
            ArmCartesianSpaceGoal(label=label, **values_dictionary)
            for label, values_dictionary in cartesian_dictionary.items()
        ]
        joint_space_goals = [
            ArmJointSpaceGoal(label=label, **values_dictionary)
            for label, values_dictionary in joint_dictionary.items()
        ]

        return cls(
            cartesian_space_goals=cartesian_space_goals,
            joint_space_goals=joint_space_goals,
        )