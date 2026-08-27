import { alertsRef, settingsRef } from './refs';
import { ros } from './ros';
import { ArmAxesLocks, JointState, SetFeedRequest } from './ros-interfaces';
import { getKeybind } from './keybinds';
import { Service, Topic } from 'roslib';

let setLinearScale: Topic<unknown> | null = null;
let setRotationalScale: Topic<unknown> | null = null;

let abortPoseTopic: Topic<unknown> | null = null;
let executePoseTopic: Topic<unknown> | null = null;
let executePoseSpecialTopic: Topic<unknown> | null = null;
let keepAlivePoseTopic: Topic<unknown> | null = null;
let statusPoseTopic: Topic<unknown> | null = null;

let abortTrajectoryTopic: Topic<unknown> | null = null;
let executeTrajectoryTopic: Topic<unknown> | null = null;
let keepAliveTrajectoryTopic: Topic<unknown> | null = null;
let statusTrajectoryTopic: Topic<unknown> | null = null;

let armAxesLocksTopic: Topic<ArmAxesLocks> | null = null;

let lastServoLinearScale: number | null = 0.5;
let lastServoRotationalScale: number | null = 0.5;
let lastStatusPose: string = 'UNKNOWN';
let lastStatusTrajectory: string = 'UNKNOWN';

let rotationalLock = false;

export const WARN_THRESHOLD = (20 * Math.PI) / 180;
export const ERROR_THRESHOLD = (5 * Math.PI) / 180;

export const CUSTOM_POSES_KEY = 'custom_arm_poses';
export const HIDDEN_POSES_KEY = 'hidden_arm_poses';
export const KEEP_ALIVE_STATUSES = ['GOAL_ACCEPTED', 'GOAL_SENDING'];

export interface ArmPose {
  id: number;
  name: string;
  joints: number[];
  isSpecial?: boolean;
  isCustom?: boolean;
}

export const jointLimitsRad = [
  { min: -6.2815926, max: 6.2815926 },
  { min: -3.1415926, max: 3.1415926 },
  { min: -2.88, max: 2.88 },
  { min: -6.4, max: 6.4 },
  { min: -1.75, max: 1.75 },
  { min: -3.4032, max: 3.4032 }
];
export function rad2deg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function isCloseEnough(
  jointsA: number[],
  jointsB: number[],
  maxDistance: number,
  checkedJoints: number[] = []
): boolean {
  if (jointsA.length !== jointsB.length) return false;
  for (let i = 0; i < jointsA.length; i++) {
    if (checkedJoints.includes(i + 1) && Math.abs(jointsA[i] - jointsB[i]) > maxDistance) {
      return false;
    }
  }
  return true;
}
export const ARM_JOINT_NAMES = [
  'arm_joint_1',
  'arm_joint_2',
  'arm_joint_3',
  'arm_joint_4',
  'arm_joint_5',
  'arm_joint_6'
];

export let lastJointState: JointState | null = null;
export function getNamesAndValues() {
  const names = lastJointState?.name ?? [];
  const positions = lastJointState?.position ?? [];

  return names
    .map((name, i) => ({
      name,
      value: positions[i] ?? 0
    }))
    .sort((a, b) => a.name.localeCompare(b.name))
    .slice(0, 6);
}

window.addEventListener('ros-connect', () => {
  const jointTopic = new Topic({
    ros: ros,
    name: '/arm_controllers/joint_states',
    messageType: 'sensor_msgs/JointState'
  });

  jointTopic.subscribe((message) => {
    const msg = message as JointState;
    lastJointState = msg;
    window.dispatchEvent(new CustomEvent('joint-state'));
  });
});

export let armAxesLocks: ArmAxesLocks = {
  x: false,
  y: false,
  z: false,
  roll: false,
  pitch: false,
  yaw: false
};

const ARM_STATUSES: Record<number, string> = {
  0: 'SUCCESS',
  1: 'CANCELLING',
  2: 'CANCEL_SUCCESS',
  3: 'CANCEL_FAILED',
  4: 'ABORT_RECEIVED',
  5: 'GOAL_ACCEPTED',
  6: 'GOAL_REJECTED',
  7: 'FAILED',
  8: 'PREEMPTING',
  9: 'INVALID_ID',
  10: 'TOO_FAR',
  11: 'EXCEPTION',
  12: 'GOAL_SENDING',
  255: 'IDLE'
};

// ROS
window.addEventListener('ros-connect', () => {
  setLinearScale = new Topic({
    ros: ros,
    name: '/servo/set_linear_scale',
    messageType: 'std_msgs/Float64'
  });

  setRotationalScale = new Topic({
    ros: ros,
    name: '/servo/set_rotational_scale',
    messageType: 'std_msgs/Float64'
  });

  abortPoseTopic = new Topic({
    ros: ros,
    name: '/pose_request/abort',
    messageType: 'example_interfaces/msg/Empty'
  });

  executePoseTopic = new Topic({
    ros: ros,
    name: '/pose_request/execute',
    messageType: 'sensor_msgs/msg/JointState'
  });

  executePoseSpecialTopic = new Topic({
    ros: ros,
    name: '/pose_request/execute_special',
    messageType: 'kalman_interfaces/ArmPoseSelect'
  });

  keepAlivePoseTopic = new Topic({
    ros: ros,
    name: '/pose_request/keep_alive',
    messageType: 'example_interfaces/msg/Empty'
  });

  statusPoseTopic = new Topic({
    ros: ros,
    name: '/pose_request/status',
    messageType: 'kalman_interfaces/ArmGoalStatus'
  });

  abortTrajectoryTopic = new Topic({
    ros: ros,
    name: '/trajectory/abort',
    messageType: 'example_interfaces/msg/Empty'
  });

  executeTrajectoryTopic = new Topic({
    ros: ros,
    name: '/trajectory/execute',
    messageType: 'kalman_interfaces/ArmTrajectorySelect'
  });

  keepAliveTrajectoryTopic = new Topic({
    ros: ros,
    name: '/trajectory/keep_alive',
    messageType: 'example_interfaces/msg/Empty'
  });

  statusTrajectoryTopic = new Topic({
    ros: ros,
    name: '/trajectory/status',
    messageType: 'kalman_interfaces/ArmGoalStatus'
  });

  armAxesLocksTopic = new Topic({
    ros: ros,
    name: '/arm/axes_locks',
    messageType: 'kalman_interfaces/ArmAxesLocks'
  });

  setLinearScale.subscribe((message) => {
    const msg = message as {data: number};
    lastServoLinearScale = msg.data;
    window.dispatchEvent(new CustomEvent('servo-linear-scale'));
  });

  setRotationalScale.subscribe((message) => {
    const msg = message as {data: number};
    lastServoRotationalScale = msg.data;
    window.dispatchEvent(new CustomEvent('servo-rotational-scale'));
  });

  statusPoseTopic.subscribe((message) => {
    const msg = message as { status: number };
    lastStatusPose = ARM_STATUSES[msg.status];
    window.dispatchEvent(new CustomEvent('pose-status'));
  });

  statusTrajectoryTopic.subscribe((message) => {
    const msg = message as { status: number };
    lastStatusTrajectory = ARM_STATUSES[msg.status];
    window.dispatchEvent(new CustomEvent('trajectory-status'));
  });

  setLinearScaleTo(lastServoLinearScale ?? 0);
  setRotationalScaleTo(lastServoRotationalScale ?? 0);
  armAxesLocksTopic?.publish(armAxesLocks);
});

function setLinearScaleTo(value: number) {
  if (setLinearScale) {
    setLinearScale.publish({ data: value });
    lastServoLinearScale = value;
  }
}

function setRotationalScaleTo(value: number) {
  if (setRotationalScale) {
    setRotationalScale.publish({ data: value });
    lastServoRotationalScale = value;
  }
}

function abortPose() {
  if (abortPoseTopic) {
    abortPoseTopic.publish({});
  }
}

function buildJointStateMsg(pose: ArmPose): JointState {
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    name: [...ARM_JOINT_NAMES],
    position: pose.joints.slice(0, 6),
    velocity: [],
    effort: []
  };
}

function sendPoseRequest(pose: ArmPose) {
  if (pose.isSpecial) {
    sendPoseSpecialRequest(pose.id);
    return;
  }

  if (executePoseTopic) {
    const jointStateMsg = buildJointStateMsg(pose);
    executePoseTopic.publish(jointStateMsg);
  }
}

function sendPoseSpecialRequest(id: number) {
  if (executePoseSpecialTopic) {
    executePoseSpecialTopic.publish({ pose_id: id });
  }
}

function keepAlivePose() {
  if (keepAlivePoseTopic) {
    keepAlivePoseTopic.publish({});
  }
}

function abortTrajectory() {
  if (abortTrajectoryTopic) {
    abortTrajectoryTopic.publish({});
  }
}

function sendTrajectoryRequest(id: number) {
  if (executeTrajectoryTopic) {
    executeTrajectoryTopic.publish({ trajectory_id: id });
  }
}

function keepAliveTrajectory() {
  if (keepAliveTrajectoryTopic) {
    keepAliveTrajectoryTopic.publish({});
  }
}

export function toggleArmAxisLock(axis: keyof ArmAxesLocks) {
  armAxesLocks[axis] = !armAxesLocks[axis];
  armAxesLocksTopic?.publish(armAxesLocks);
  window.dispatchEvent(new CustomEvent('arm-axis-lock-update'));
}

window.addEventListener('keydown', (event) => {
  // Check if any input box is focused.
  if (document.activeElement?.tagName === 'INPUT') {
    return;
  }
  // Check if settings are open.
  if (settingsRef.current?.isShown()) {
    return;
  }

  switch (event.code) {
    case getKeybind('Set SpaceMouse Linear Scale to Slow (0.1)'):
      setLinearScaleTo(0.1);
      break;
    case getKeybind('Set SpaceMouse Linear Scale to Medium (0.3)'):
      setLinearScaleTo(0.3);
      break;
    case getKeybind('Set SpaceMouse Linear Scale to Fast (0.7)'):
      setLinearScaleTo(0.7);
      break;
    case getKeybind('Set SpaceMouse Rotational Scale to Slow (0.1)'):
      setRotationalScaleTo(0.1);
      break;
    case getKeybind('Set SpaceMouse Rotational Scale to Medium (0.3)'):
      setRotationalScaleTo(0.3);
      break;
    case getKeybind('Set SpaceMouse Rotational Scale to Fast (0.7)'):
      setRotationalScaleTo(0.7);
      break;
    case getKeybind('Lock SpaceMouse X Axis'):
      if (rotationalLock) {
        toggleArmAxisLock('roll');
      } else {
        toggleArmAxisLock('x');
      }
      break;
    case getKeybind('Lock SpaceMouse Y Axis'):
      if (rotationalLock) {
        toggleArmAxisLock('pitch');
      } else {
        toggleArmAxisLock('y');
      }
      break;
    case getKeybind('Lock SpaceMouse Z Axis'):
      if (rotationalLock) {
        toggleArmAxisLock('yaw');
      } else {
        toggleArmAxisLock('z');
      }
      break;

    case getKeybind('Hold to Lock Rotational Axes'):
      rotationalLock = true;
      break;
  }
});

window.addEventListener('keyup', (event) => {
  if (event.code === getKeybind('Hold to Lock Rotational Axes')) {
    rotationalLock = false;
  }
});

export {
  setLinearScaleTo,
  setRotationalScaleTo,
  lastServoLinearScale,
  lastServoRotationalScale,
  abortPose,
  sendPoseRequest,
  sendPoseSpecialRequest,
  keepAlivePose,
  lastStatusPose,
  abortTrajectory,
  sendTrajectoryRequest,
  keepAliveTrajectory,
  lastStatusTrajectory
};
