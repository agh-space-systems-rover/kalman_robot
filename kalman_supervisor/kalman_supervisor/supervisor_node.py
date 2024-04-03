import rclpy
import time
import traceback

from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from kalman_supervisor.module import Module
from kalman_supervisor.state import State, disabled_state_classes
from kalman_supervisor.modules import *
from kalman_supervisor.states import *


def all_inherited_classes(cls):
    return cls.__subclasses__() + [
        g for s in cls.__subclasses__() for g in all_inherited_classes(s)
    ]


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")

        # Set per-module type hints for autocomplete.
        self.aruco: ArUco
        self.map: Map
        self.missions: Missions
        self.nav: Nav
        self.position_history: PositionHistory
        self.tf: TF
        self.ueuos: Ueuos

        # Initialize modules.
        module_classes = all_inherited_classes(Module)
        self.__modules = [module_class() for module_class in module_classes]
        for module in self.__modules:
            module.supervisor = self
            # Set self.missions, self.nav, etc.
            self.__setattr__(module.name, module)

        # Initialize states.
        # state_dict maps state names to State objects.
        state_classes = all_inherited_classes(State)
        state_classes = [
            state_class
            for state_class in state_classes
            if state_class not in disabled_state_classes
        ]
        self.__state_dict = [state_class() for state_class in state_classes]
        self.__state_dict = {state.name: state for state in self.__state_dict}
        for state_obj in self.__state_dict.values():
            state_obj.supervisor = self

        # Auto-configure.
        result = self.trigger_configure()
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("Failed to auto-configure.")
            return

        # Auto-activate.
        result = self.trigger_activate()
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("Failed to auto-activate.")
            return

    def on_configure(self, _) -> TransitionCallbackReturn:
        try:
            # Declare parameters.
            self.declare_parameter("rate", 3.0)

            # Configure modules.
            for module in self.__modules:
                module.configure()
        except Exception as e:
            self.get_logger().error(
                f"Error during configuration:\n{traceback.format_exc()}"
            )
            return TransitionCallbackReturn.ERROR

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _) -> TransitionCallbackReturn:
        try:
            # Get parameters.
            rate = self.get_parameter("rate").value

            # Activate modules.
            for module in self.__modules:
                module.activate()

            # Enter initial state.
            self.state = "teleop"
            self.__state_dict[self.state].enter()

            # Start ticking.
            self.__tick_timer = self.create_timer(1 / rate, self.tick)
        except Exception as e:
            self.get_logger().error(
                f"Error during activation:\n{traceback.format_exc()}"
            )
            return TransitionCallbackReturn.ERROR

        return TransitionCallbackReturn.SUCCESS

    def tick(self) -> None:
        # Tick modules.
        # If a tick fails, log the error and continue to the next one.
        # The failure should never happen, but we cannot stop during a mission.
        for module in self.__modules:
            try:
                module.tick()
            except Exception as e:
                self.get_logger().error(
                    f"Error when ticking {module.name}:\n{traceback.format_exc()}\nContinuing to next module. Let there be dragons."
                )

        # Tick the state.
        new_state_name = None
        try:
            new_state_name = self.__state_dict[self.state].tick()
        except Exception as e:
            self.get_logger().error(
                f"Error when ticking {self.state} state:\n{traceback.format_exc()}\nContinuing to next tick. Let there be dragons."
            )

        # Transition if requested.
        if new_state_name and new_state_name != self.state:
            try:
                self.__state_dict[self.state].exit()
            except Exception as e:
                self.get_logger().error(
                    f"Error when exiting {self.state} state:\n{traceback.format_exc()}\nContinuing to next tick. Let there be dragons."
                )
            self.state = new_state_name
            try:
                self.__state_dict[self.state].enter()
            except Exception as e:
                self.get_logger().error(
                    f"Error when entering {self.state} state:\n{traceback.format_exc()}\nContinuing to next tick. Let there be dragons."
                )
            self.get_logger().info(f"[State] Transitioned to {new_state_name}.")

    def on_deactivate(self, _) -> TransitionCallbackReturn:
        self.destroy_timer(self.__tick_timer)
        self.__state_dict[self.state].exit()
        self.state = None
        for module in self.__modules:
            module.deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=10)
        # num_threads = max number of concurrent mission requests + 1

        supervisor = Supervisor()
        executor.add_node(supervisor)
        executor.spin()
        supervisor.destroy_node()

        executor.shutdown()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
