# AUTO-GENERATED FILE. DO NOT EDIT.
# SEE: kalman_bringup/tools/gen_type_hints

from typing import Literal, TypedDict


class Yolo(TypedDict):
    rgbd_ids: str
    config: str


class UnitySim(TypedDict):
    scene: str


class Nav2(TypedDict):
    component_container: str
    rgbd_ids: str
    static_map: str


class Slam(TypedDict):
    component_container: str
    rgbd_ids: str
    gps_datum: str
    fiducials: str


class Gs(TypedDict):
    pass


class Clouds(TypedDict):
    component_container: str
    rgbd_ids: str


class Wheels(TypedDict):
    joy: str


class Supervisor(TypedDict):
    aruco_rgbd_ids: str
    aruco_deactivate_unused: str
    yolo_enabled: str
    yolo_deactivate_unused: str


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
    master: str
    rgbd_ids: str
    imu: str
    compass_calibration: str
    gps: str


class Description(TypedDict):
    layout: Literal["", "autonomy", "arm"]
    joint_state_publisher_gui: Literal["true", "false"]


class Aruco(TypedDict):
    component_container: str
    rgbd_ids: str
    dict: str
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
