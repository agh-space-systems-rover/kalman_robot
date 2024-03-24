from typing import Tuple, List
from rclpy.node import Node
from supervisor.srv import (
    SetWaypoints,
    SetWaypointsResponse,
    SetWaypointsRequest,
    GetWaypoints,
    GetWaypointsResponse,
    GetWaypointsRequest,
)


Vec2 = Tuple[float, float]
node = Node()

def flatten(array: List[Vec2]) -> List[float]:
    return list(sum(array, ()))


class WaypointsServer:
    def __init__(self):
        self.__waypoints: List[Vec2] = []
        self.__get_srv = node.create_service(GetWaypoints,"/waypoints/get", self.__get_callback)
        self.__set_srv = node.create_service(SetWaypoints, "/waypoints/set", self.__set_callback)
        
    @property
    def waypoints(self) -> List[Vec2]:
        return self.__waypoints

    def __get_callback(self, req: GetWaypointsRequest):
        return GetWaypointsResponse(waypoints=flatten(self.__waypoints))

    def __set_callback(self, req: SetWaypointsRequest):
        n = len(req.waypoints)

        # When the given list of waypoints contains an invalid number of coordinates, empty the list
        if n % 2 != 0 or n < 2:
            self.__waypoints = []
        else:
            self.__waypoints = [tuple(req.waypoints[i : i + 2]) for i in range(0, n, 2)]
        return SetWaypointsResponse()