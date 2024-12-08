# AUTO-GENERATED FILE. DO NOT EDIT.
# SEE: kalman_bringup/tools/gen_type_hints

from typing import Literal, TypedDict


class Yolo(TypedDict):
    rgbd_ids: str
    config: Literal["urc2024"]


class UnitySim(TypedDict):
    scene: str


class Nav2(TypedDict):
    component_container: str
    rgbd_ids: str
    static_map: Literal["", "erc2023", "erc2024"]


class Slam(TypedDict):
    component_container: str
    rgbd_ids: str
    gps_datum: str
    fiducials: Literal["", "terc2024", "erc2024"]


class Gs(TypedDict):
    pass


class Clouds(TypedDict):
    component_container: str
    rgbd_ids: str


class Wheels(TypedDict):
    joy: Literal["", "gamepad", "arduino"]


class Supervisor(TypedDict):
    aruco_rgbd_ids: str
    aruco_deactivate_unused: Literal["true", "false"]
    yolo_enabled: Literal["true", "false"]
    yolo_deactivate_unused: Literal["true", "false"]


class Spacenav(TypedDict):
    pass


class Rviz(TypedDict):
    configs: str


class Master(TypedDict):
    mode: Literal["gs", "pc", "arm"]


class ArmUtils(TypedDict):
    pass


class Hardware(TypedDict):
    component_container: str
    master: Literal["pc", "gs", "arm"]
    rgbd_ids: str
    imu: Literal["true", "false"]
    compass_calibration: str
    gps: Literal["true", "false"]


class Description(TypedDict):
    layout: Literal["autonomy", "arm"]
    joint_state_publisher_gui: Literal["true", "false"]


class Aruco(TypedDict):
    component_container: str
    rgbd_ids: str
    dict: Literal["4X4_50", "4X4_100", "4X4_250", "4X4_1000", "5X5_50", "5X5_100", "5X5_250", "5X5_1000", "6X6_50", "6X6_100", "6X6_250", "6X6_1000", "7X7_50", "7X7_100", "7X7_250", "7X7_1000", "8X8_50", "8X8_100", "8X8_250", "8X8_1000", "ARUCO_ORIGINAL"]
    size: str


class BringupConfig(TypedDict):
    yolo: Yolo
    unity_sim: UnitySim
    nav2: Nav2
    slam: Slam
    gs: Gs
    clouds: Clouds
    wheels: Wheels
    supervisor: Supervisor
    spacenav: Spacenav
    rviz: Rviz
    master: Master
    arm_utils: ArmUtils
    hardware: Hardware
    description: Description
    aruco: Aruco
