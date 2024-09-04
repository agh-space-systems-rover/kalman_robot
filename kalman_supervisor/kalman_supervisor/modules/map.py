import numpy as np

from enum import Enum
from rclpy import Future
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType, Parameter

from kalman_supervisor.module import Module


class Map(Module):
    class Occupancy(Enum):
        OUT_OF_BOUNDS = 0
        UNKNOWN = 1
        FREE = 2
        PARTIALLY_OCCUPIED = 3
        OCCUPIED = 4

    def __init__(self):
        super().__init__("map")

    def configure(self) -> None:
        self.supervisor.declare_parameter("map.obstacle_layers", ["static_layer"])

    def activate(self) -> None:
        self.obstacle_layers = self.supervisor.get_parameter(
            "map.obstacle_layers"
        ).value

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        update_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.__map_sub = self.supervisor.create_subscription(
            OccupancyGrid, "map/map", self.__map_cb, map_qos
        )
        self.__map_update_sub = self.supervisor.create_subscription(
            OccupancyGridUpdate, "map/map_updates", self.__map_update_cb, update_qos
        )

        self.__frame: str | None = None
        self.__resolution: float | None = None
        self.__origin_np: np.ndarray | None = None
        self.__data_np: np.ndarray | None = None

        self.__global_param_set_client = self.supervisor.create_client(
            SetParameters, "map/global/set_parameters"
        )
        self.__global_param_set_future: Future | None = None
        self.__local_param_set_client = self.supervisor.create_client(
            SetParameters, "map/local/set_parameters"
        )
        self.__local_param_set_future: Future | None = None

        # # Debug: republish own map.
        # self.__map_pub = self.supervisor.create_publisher(OccupancyGrid, "map/map_copy", map_qos)

        self.__obstacle_layers_enabled = False
        self.__obstacle_layers_should_be_enabled = True
        # Setting them to different values like this triggers a request to enable layers on activation.

    def tick(self) -> None:
        # If the obstacle layers should be enabled but they are
        # not or vice versa, and a change request is not pending, request the parameters to change.
        if self.__obstacle_layers_enabled != self.__obstacle_layers_should_be_enabled:
            if (
                self.__global_param_set_future is None
                and self.__local_param_set_future is None
            ):
                req = SetParameters.Request()
                for layer in self.obstacle_layers:
                    param = Parameter()
                    param.name = f"{layer}.enabled"
                    param.value.type = ParameterType.PARAMETER_BOOL
                    param.value.bool_value = self.__obstacle_layers_should_be_enabled
                    req.parameters.append(param)
                self.__global_param_set_future = (
                    self.__global_param_set_client.call_async(req)
                )
                self.__local_param_set_future = (
                    self.__local_param_set_client.call_async(req)
                )

        # Reset param futures once both are done.
        if (
            self.__global_param_set_future is not None
            and self.__global_param_set_future.done()
            and self.__local_param_set_future is not None
            and self.__local_param_set_future.done()
        ):
            self.__global_param_set_future = None
            self.__local_param_set_future = None
            # The request is only ever sent in order to flip the state, so it is safe to
            # assume that the new state will be opposite to the old one.
            self.__obstacle_layers_enabled = not self.__obstacle_layers_enabled
            self.supervisor.get_logger().info(
                f"[Nav2/Costmap2D] Obstacle layers are now {'enabled' if self.__obstacle_layers_enabled else 'disabled'}."
            )

        # # Debug: republish own map.
        # if self.__data_np is not None:
        #     msg = OccupancyGrid()
        #     msg.header.frame_id = self.__frame
        #     msg.info.resolution = self.__resolution
        #     msg.info.width = self.__data_np.shape[1]
        #     msg.info.height = self.__data_np.shape[0]
        #     msg.info.origin.position.x = self.__origin_np[0]
        #     msg.info.origin.position.y = self.__origin_np[1]
        #     msg.data = self.__data_np.flatten().tolist()
        #     self.__map_pub.publish(msg)

    def deactivate(self) -> None:
        self.supervisor.destroy_subscription(self.__map_update_sub)
        self.supervisor.destroy_subscription(self.__map_sub)

    def __map_cb(self, msg: OccupancyGrid) -> None:
        self.__frame = msg.header.frame_id
        self.__resolution = msg.info.resolution
        self.__origin_np = np.array(
            [msg.info.origin.position.x, msg.info.origin.position.y]
        )
        self.__data_np = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def __map_update_cb(self, msg: OccupancyGridUpdate) -> None:
        x0 = msg.x
        y0 = msg.y
        x1 = x0 + msg.width
        y1 = y0 + msg.height

        # Abort if the map is not received yet.
        if self.__data_np is None:
            return

        # Abort if the update is out of bounds. (Should never happen though.)
        if (
            x0 < 0
            or y0 < 0
            or x1 > self.__data_np.shape[1]
            or y1 > self.__data_np.shape[0]
        ):
            return

        data = np.array(msg.data).reshape(msg.height, msg.width)
        self.__data_np[y0:y1, x0:x1] = data

    def __pos_to_grid_coords(self, pos: np.ndarray, frame: str) -> np.ndarray:
        pos = pos.copy()
        if frame != self.__frame:
            pos = self.supervisor.tf.transform_numpy(pos, self.__frame, frame)

        xy = pos[:2]
        xy -= self.__origin_np
        xy /= self.__resolution
        xy += 0.5 * np.sign(xy)  # round to nearest int in astype
        xy = xy.astype(int)
        return xy

    def __grid_coords_to_pos(self, xy: np.ndarray) -> tuple[np.ndarray, str]:
        xy = xy.copy()
        xy = xy.astype(float)
        xy *= self.__resolution
        xy += self.__origin_np
        return xy, self.__frame

    def __occupancy_for_grid_coords(self, xy: np.ndarray) -> Occupancy:
        if np.any(xy < 0) or np.any(xy >= np.array(self.__data_np.shape)):
            return Map.Occupancy.OUT_OF_BOUNDS

        value = self.__data_np[xy[1], xy[0]]

        if value == -1:
            return Map.Occupancy.UNKNOWN
        elif value == 0:
            return Map.Occupancy.FREE
        elif value < 50:
            return Map.Occupancy.PARTIALLY_OCCUPIED
        else:
            return Map.Occupancy.OCCUPIED

    def frame(self) -> str | None:
        return self.__frame

    def occupancy(self, pos: np.ndarray, frame: str) -> Occupancy:
        if self.__frame is None:
            raise RuntimeError("Map was not received yet.")

        pos = pos.copy()
        xy = self.__pos_to_grid_coords(pos, frame)
        return self.__occupancy_for_grid_coords(xy)

    # Find the closest free position to the given position.
    # The returned position has the same dimensionality as the input position.
    # The frame of the returned position is the same as the input frame.
    # A 3D position will get snapped to the Z=0 plane of the world.
    # Return None if no free position was found with the hard-coded tolerance.
    def closest_free(self, pos: np.ndarray, frame: str) -> np.ndarray | None:
        if self.__frame is None:
            raise RuntimeError("Map was not received yet.")

        pos = pos.copy()
        xy = self.__pos_to_grid_coords(
            pos, frame
        )  # height over the map frame is discarded

        # First check if the point is out of bounds.
        if self.__occupancy_for_grid_coords(xy) == Map.Occupancy.OUT_OF_BOUNDS:
            # In this case clamp it to the bounds.
            # This is the closest point within the bounds.
            xy = np.clip(xy, 1, np.array(self.__data_np.shape) - 2)

        # Do an outward spiral search.
        side_index = 0
        steps_for_this_side = 1
        dir = np.array([1, 0])
        samples = 0
        while True:
            # If the point is free, return it.
            if (
                self.__occupancy_for_grid_coords(xy) == Map.Occupancy.FREE
                or self.__occupancy_for_grid_coords(xy)
                == Map.Occupancy.PARTIALLY_OCCUPIED
            ):
                xy, new_frame = self.__grid_coords_to_pos(xy)
                # The 3D point will be snapped to the Z=0 plane in the map frame.
                pos = np.append(xy, [0])
                xy = self.supervisor.tf.transform_numpy(pos, frame, new_frame)
                return xy

            # Advance the spiral.
            xy += dir
            steps_for_this_side -= 1
            if steps_for_this_side == 0:
                side_index += 1
                steps_for_this_side = side_index // 2 + 1
                if np.all(dir == np.array([1, 0])):
                    dir = np.array([0, 1])
                elif np.all(dir == np.array([0, 1])):
                    dir = np.array([-1, 0])
                elif np.all(dir == np.array([-1, 0])):
                    dir = np.array([0, -1])
                else:
                    dir = np.array([1, 0])

            # Count the number of samples to eventually fail.
            samples += 1
            if samples > 10000:
                break

        return None

    # Specification analogous to closest_free.
    # Dir is in the same frame as pos. It is automatically normalized.
    def closest_free_in_dir(
        self, pos: np.ndarray, frame: str, dir: np.ndarray
    ) -> np.ndarray | None:
        if self.__frame is None:
            raise RuntimeError("Map was not received yet.")

        pos = pos.copy()
        xy = self.__pos_to_grid_coords(
            pos, frame
        )  # height over the map frame is discarded
        xy_dir = self.supervisor.tf.rotate_numpy(dir, self.__frame, frame)[:2]

        # First check if the point is out of bounds.
        if np.any(xy < 0) or np.any(xy >= np.array(self.__data_np.shape)):
            # In this case project it onto the center of the map and start searching from the edge.
            # TODO: Should be projected along the direction, but this is good enough for now since robot is always in the center.
            center_xy = np.array(self.__data_np.shape) // 2
            from_center = xy - center_xy
            scale_xy = np.abs((np.array(self.__data_np.shape) // 2) / from_center)
            scale = np.min(scale_xy)
            xy = center_xy + (from_center * scale).astype(int)
            xy = np.clip(
                xy, 1, np.array(self.__data_np.shape) - 2
            )  # make sure that xy is in bounds

        # Do a linear search along the vector.
        xy = xy.astype(float)
        xy_dir /= np.linalg.norm(xy_dir)
        while True:
            xy_int = (xy + 0.5).astype(int)
            if self.__occupancy_for_grid_coords(xy_int) == Map.Occupancy.FREE:
                xy, new_frame = self.__grid_coords_to_pos(xy)
                # The 3D point will be snapped to the Z=0 plane in the map frame.
                pos = np.append(xy, [0])
                xy = self.supervisor.tf.transform_numpy(pos, frame, new_frame)
                return xy

            # Advance along the vector and break out if we went out of bounds.
            xy += xy_dir
            if np.any(xy < 0) or np.any(xy >= self.__data_np.shape):
                break

        return None

    def closest_in_bounds_towards_center(
        self, pos: np.ndarray, frame: str
    ) -> np.ndarray:
        if self.__frame is None:
            raise RuntimeError("Map was not received yet.")

        pos = pos.copy()
        xy = self.__pos_to_grid_coords(pos, frame)

        if np.any(xy < 0) or np.any(xy >= np.array(self.__data_np.shape)):
            # In this case project it onto the center of the map and start searching from the edge.
            # TODO: Should be projected along the direction, but this is good enough for now since robot is always in the center.
            center_xy = np.array(self.__data_np.shape) // 2
            from_center = xy - center_xy
            scale_xy = np.abs((np.array(self.__data_np.shape) // 2) / from_center)
            scale = np.min(scale_xy)
            xy = center_xy + (from_center * scale).astype(int)
            xy = np.clip(
                xy, 1, np.array(self.__data_np.shape) - 2
            )  # make sure that xy is in bounds

        xy, new_frame = self.__grid_coords_to_pos(xy)
        pos = np.append(xy, [0])
        xy = self.supervisor.tf.transform_numpy(pos, frame, new_frame)
        return xy

    def obstacle_layers_enabled(self) -> bool:
        return self.__obstacle_layers_enabled

    # Wait for obstacle_layers_enabled to know when it's done.
    def disable_obstacle_layers(self) -> None:
        self.__obstacle_layers_should_be_enabled = False

    # Wait for obstacle_layers_enabled to know when it's done.
    def enable_obstacle_layers(self) -> None:
        self.__obstacle_layers_should_be_enabled = True

    def metric_size(self) -> np.ndarray:
        if self.__frame is None:
            raise RuntimeError("Map was not received yet.")

        return np.array(self.__data_np.shape) * self.__resolution
