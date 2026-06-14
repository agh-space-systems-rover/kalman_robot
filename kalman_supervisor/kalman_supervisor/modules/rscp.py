import math
import utm
import numpy as np

from kalman_supervisor.module import Module
from kalman_interfaces.msg import ArcRscpRequest, ArcRscpResponse


class Rscp(Module):
    def __init__(self):
        super().__init__("rscp")
        self.__autonomous_from_ueuos = False

    def configure(self) -> None:
        self.supervisor.declare_parameter("rscp.enabled", False)

    def activate(self) -> None:
        self.module_enabled = self.supervisor.get_parameter("rscp.enabled").value
        if not self.module_enabled:
            return

        self.__armed = False
        self.__current_stage: int | None = None
        self.__pending_request: ArcRscpRequest | None = None
        self.__navigation_goal: tuple[float, float] | None = None  # (lat, lon)
        self.__search_goal: tuple[float, float] | None = None  # (lat, lon)
        self.__exploration_start_requested = False

        # Subscribe to rscp/req
        self.__req_sub = self.supervisor.create_subscription(
            ArcRscpRequest, "rscp/req", self.__req_callback, 10
        )

        # Publisher for rscp/res
        self.__res_pub = self.supervisor.create_publisher(
            ArcRscpResponse, "rscp/res", 10
        )

    def __req_callback(self, msg: ArcRscpRequest) -> None:
        # Store the request for processing
        self.__pending_request = msg
        self.supervisor.get_logger().info(
            f"[RSCP] Received request type={msg.type} "
            f"(arm={msg.arm}, stage={msg.stage}, lat={msg.latitude}, lon={msg.longitude})"
        )

    def tick(self) -> None:
        if not self.module_enabled:
            return

        # Process all request types in the module before states see them
        if self.__pending_request is not None:
            req = self.__pending_request
            self.__pending_request = None  # Consume the request

            # Handle ARM_DISARM requests immediately
            if req.type == ArcRscpRequest.ARM_DISARM:
                self.__armed = req.arm
                self.send_ack()

                self.supervisor.get_logger().info(
                    f"[RSCP] ARM_DISARM: {'ARMED' if self.__armed else 'DISARMED'}"
                )

                # Update UEUOS state based on armed status
                if self.__armed:
                    self.supervisor.ueuos.set_rscp_state(
                        self.supervisor.ueuos.RscpState.ARMED
                    )
                else:
                    self.supervisor.ueuos.set_rscp_state(
                        self.supervisor.ueuos.RscpState.DISARMED
                    )

            # Handle SET_STAGE requests immediately
            elif req.type == ArcRscpRequest.SET_STAGE:
                self.__current_stage = req.stage
                self.send_ack()

                self.supervisor.get_logger().info(
                    f"[RSCP] SET_STAGE: stage={self.__current_stage}"
                )

            elif req.type == ArcRscpRequest.NAV_TO_GPS:
                if not self.is_armed():
                    self.supervisor.get_logger().warn(
                        "[RSCP] NavigateToGPS rejected: rover is DISARMED"
                    )
                else:
                    self.__navigation_goal = (req.latitude, req.longitude)
                    self.send_ack()
                    self.supervisor.get_logger().info(
                        "[RSCP] NavigateToGPS received "
                        f"(lat={req.latitude}, lon={req.longitude}), sent ACK"
                    )
            
            elif req.type == ArcRscpRequest.SEARCH_AREA:
                if not self.is_armed():
                    self.supervisor.get_logger().warn(
                        "[RSCP] SearchArea request rejected: rover is DISARMED"
                    )
                else:
                    self.supervisor.get_logger().warn(
                        "[RSCP] SearchArea request received but not implemented"
                    )
                    self.__search_goal = (req.latitude, req.longitude)
                    self.send_ack()
                    self.supervisor.get_logger().info(
                        "[RSCP] Seach_AREA received "
                        f"(lat={req.latitude}, lon={req.longitude}), sent ACK"
                    )

            elif req.type == ArcRscpRequest.START_EXPLORATION:
                self.__exploration_start_requested = True
                self.send_ack()
                self.supervisor.get_logger().info(
                    "[RSCP] START_EXPLORATION received, sent ACK"
                )

    def deactivate(self) -> None:
        if not self.module_enabled:
            return

        self.supervisor.destroy_subscription(self.__req_sub)
        self.supervisor.destroy_publisher(self.__res_pub)

    def is_armed(self) -> bool:
        return self.__armed

    def send_ack(self) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.ACK
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info("[RSCP] Sent ACK")

    def send_task_finished(self) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.TASK_FINISHED
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info("[RSCP] Sent TASK_FINISHED")

    def send_distance(self, distance: float) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.DISTANCE
        msg.distance = float(distance)
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info(f"[RSCP] Sent DISTANCE: {distance}")

    def send_gps_coordinate(
        self, latitude: float, longitude: float, altitude: float = 0.0
    ) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.GPS_COORDINATE
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info(
            f"[RSCP] Sent GPS coordinate (lat={latitude}, lon={longitude}, alt={altitude})"
        )

    def send_message(self, text: str) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.MESSAGE
        msg.message = text
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info(f"[RSCP] Sent message: {text}")

    def send_rover_status(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Sending rover status")
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.ROVER_STATUS

        def get_rover_state() -> int:
            if self.is_armed():
                if self.__autonomous_from_ueuos:
                    return ArcRscpResponse.ROVER_STATE_AUTONOMOUS
                else:
                    return ArcRscpResponse.ROVER_STATE_MANUAL
            else:
                return ArcRscpResponse.ROVER_STATE_DISARMED

        msg.rover_state = get_rover_state()

        robot_pos = self.supervisor.tf.robot_pos("base_link")
        msg.latitude, msg.longitude = self.point_to_latlon(robot_pos, "base_link")
        msg.altitude = 0.0  # Explicitly don't set altitude, docs are silent about it

        # This is returned as [-pi, pi], but RSCP expects what?
        msg.heading = self.supervisor.tf.robot_rot_2d("utm")

        ## Battery state
        msg.voltage = 24.2
        msg.current = 3.14 * abs(
            math.sin(self.supervisor.get_clock().now().nanoseconds / 1e9)
        )
        msg.state_of_charge = 0.42

        self.__res_pub.publish(msg)

    def get_current_stage(self) -> int | None:
        return self.__current_stage

    def get_navigation_goal(self) -> tuple[float, float] | None:
        return self.__navigation_goal
    
    def get_search_goal(self) -> tuple[float, float] | None:
        return self.__search_goal
    
    def clear_search_goal(self) -> None:
        self.__search_goal = None
        self.supervisor.get_logger().info("[RSCP] Search goal cleared")

    def clear_navigation_goal(self) -> None:
        self.__navigation_goal = None
        self.supervisor.get_logger().info("[RSCP] Navigation goal cleared")

    def exploration_start_requested(self) -> bool:
        return self.__exploration_start_requested

    def clear_exploration_start_request(self) -> None:
        self.__exploration_start_requested = False
        self.supervisor.get_logger().info("[RSCP] Exploration start request cleared")
