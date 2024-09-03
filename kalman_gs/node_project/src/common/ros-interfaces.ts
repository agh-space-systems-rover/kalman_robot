// Standard ROS types

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
