# import time
# import numpy as np

# from kalman_supervisor.state import State
# from kalman_supervisor.modules import *

# # number of revolutions of the spiral per 1 progress unit
# SPIRAL_REVOLUTIONS = 3
# # the distance between the revolutions of the spiral
# SPIRAL_REVOLUTION_WIDTH = 4

# def spiral(progress: float) -> np.ndarray:
#     t = 2 * np.pi * SPIRAL_REVOLUTIONS * progress
#     r = SPIRAL_REVOLUTIONS * SPIRAL_REVOLUTION_WIDTH * progress

#     return np.array([r * np.cos(t), r * np.sin(t)])

# class Spiral(State):
#     def __init__(self):
#         super().__init__("spiral")

#     def enter(self) -> None:
#         self.progress = 0

#     def tick(self) -> str | None:
#         # Fall back to teleop if mission was ended early.
#         if not self.supervisor.missions.has_mission():
#             return "teleop"
        
#         # Calculate the next position on the spiral.


