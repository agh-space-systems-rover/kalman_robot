// Standard ROS types

export type UInt8 = {
  data: number;
};

export type Time = {
  sec?: number;
  nanosec?: number;
};

export type Header = {
  stamp?: Time;
  frame_id?: string;
};

export type NavSatStatus = {
  status?: number;
  service?: number;
};

export type NavSatFix = {
  header?: Header;
  status?: NavSatStatus;
  latitude?: number;
  longitude?: number;
  altitude?: number;
  position_covariance?: number[];
};

export type Quaternion = {
  x?: number;
  y?: number;
  z?: number;
  w?: number;
};

export type Vector3 = {
  x?: number;
  y?: number;
  z?: number;
};

export type Imu = {
  header?: Header;
  orientation?: Quaternion;
  orientation_covariance?: number[];
  angular_velocity?: Vector3;
  angular_velocity_covariance?: number[];
  linear_acceleration?: Vector3;
  linear_acceleration_covariance?: number[];
};

export type Point = {
  x?: number;
  y?: number;
  z?: number;
};

export type PointStamped = {
  header?: Header;
  point?: Point;
};

export type GeoPoint = {
  latitude?: number;
  longitude?: number;
  altitude?: number;
};

export type Twist = {
  linear?: Vector3;
  angular?: Vector3;
};

export type Bool = {
  data?: boolean;
};

export type Pose = {
  position?: Point;
  orientation?: Quaternion;
};

export type PoseWithCovariance = {
  pose?: Pose;
  covariance?: number[];
};

export type TwistWithCovariance = {
  twist?: Twist;
  covariance?: number[];
};

export type Odometry = {
  header?: Header;
  pose?: PoseWithCovariance;
  twist?: TwistWithCovariance;
};

// std_srvs/SetBool
export type SetBoolRequest = {
  data?: boolean;
};
export type SetBoolResponse = {
  success?: boolean;
  message?: string;
};

export type ColorRGBA = {
  r?: number;
  g?: number;
  b?: number;
  a?: number;
};

// kalman_interfaces

export type SupervisorTfGoal = {
  location?: PointStamped;
};
export type SupervisorTfGoalFeedback = {
  state?: string;
};
// Result is empty.

export type SupervisorGpsGoal = {
  location?: GeoPoint;
};
export type SupervisorGpsGoalFeedback = {
  state?: string;
};
// Result is empty.

export type SupervisorGpsArUcoSearch = {
  initial_location?: GeoPoint;
  marker_id?: number;
};
export type SupervisorGpsArUcoSearchFeedback = {
  state?: string;
  marker_found?: boolean;
  marker_location?: GeoPoint;
};
// Result is empty.

export type SupervisorGpsYoloSearch = {
  initial_location?: GeoPoint;
  object_class?: string;
};
export type SupervisorGpsYoloSearchFeedback = {
  state?: string;
  object_found?: boolean;
  object_location?: GeoPoint;
};
// Result is empty.

export type GeoPath = {
  points: GeoPoint[];
};

export type WheelState = {
  velocity?: number;
  angle?: number;
};
export type WheelStates = {
  front_left?: WheelState;
  front_right?: WheelState;
  back_left?: WheelState;
  back_right?: WheelState;
};
export type WheelTemperature = {
  motor?: number;
  swivel?: number;
};
export type WheelTemperatures = {
  front_left?: WheelTemperature;
  front_right?: WheelTemperature;
  back_left?: WheelTemperature;
  back_right?: WheelTemperature;
};

export type SetFeedRequest = {
  feed?: number;
  camera?: number;
  channel?: number;
  power?: number;
};
// Response is empty.

export type SpoofGpsRequest = {
  location?: GeoPoint;
};
// Response is empty.

export type ReadPkgConfigFileRequest = {
  pkg?: string;
  path?: string;
};
export type ReadPkgConfigFileResponse = {
  content?: string;
  success?: boolean;
};

export type JointState = {
  header?: Header;
  name?: string[];
  position?: number[];
  velocity?: number[];
  effort?: number[];
};

export type Drive = {
  speed?: number;
  inv_radius?: number;
  sin_angle?: number;
  rotation?: number;
};

export type ArmFkCommand = {
  gripper?: number;
  joint_1?: number;
  joint_2?: number;
  joint_3?: number;
  joint_4?: number;
  joint_5?: number;
  joint_6?: number;
};

export type ArmAxesLocks = {
  x?: boolean;
  y?: boolean;
  z?: boolean;
  roll?: boolean;
  pitch?: boolean;
  yaw?: boolean;
};

export type SetUeuosColorRequest = {
  color?: ColorRGBA;
};
// Response is empty.

export const SET_UEUOS_STATE_OFF: number = 0;
export const SET_UEUOS_STATE_AUTONOMY: number = 1;
export const SET_UEUOS_STATE_TELEOP: number = 2;
export const SET_UEUOS_STATE_FINISHED: number = 3;
export type SetUeuosStateRequest = {
  state?: number;
};
// Response is empty.

export const SET_UEUOS_EFFECT_BOOT: number = 0;
export const SET_UEUOS_EFFECT_RAINBOW: number = 1;
export type SetUeuosEffectRequest = {
  effect?: number;
};
// Response is empty.

export type ColorRGB = {
  r: number;
  g: number;
  b: number;
};
