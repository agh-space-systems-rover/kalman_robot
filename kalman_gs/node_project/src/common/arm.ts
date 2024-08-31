import { alertsRef, settingsRef } from '../common/refs';
import { ros } from '../common/ros';
import { SetFeedRequest } from '../common/ros-interfaces';
import { getKeybind } from './keybinds';
import { Service, Topic } from 'roslib';

let setLinearScale: Topic<unknown> | null = null;
let setRotationalScale: Topic<unknown> | null = null;

let abortPoseTopic: Topic<unknown> | null = null;
let executePoseTopic: Topic<unknown> | null = null;
let keepAlivePoseTopic: Topic<unknown> | null = null;
let statusPoseTopic: Topic<unknown> | null = null;

let abortTrajectoryTopic: Topic<unknown> | null = null;
let executeTrajectoryTopic: Topic<unknown> | null = null;
let keepAliveTrajectoryTopic: Topic<unknown> | null = null;
let statusTrajectoryTopic: Topic<unknown> | null = null;

let lastServoLinearScale: number | null = 0.5;
let lastServoRotationalScale: number | null = 0.5;
let lastStatusPose: string = 'UNKNOWN';
let lastStatusTrajectory: string = 'UNKNOWN';

const ARM_STATUSES = {
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

  setLinearScale.subscribe((msg: { data: number }) => {
    lastServoLinearScale = msg.data;
    window.dispatchEvent(new Event('servo-linear-scale'));
  });

  setRotationalScale.subscribe((msg: { data: number }) => {
    lastServoRotationalScale = msg.data;
    window.dispatchEvent(new Event('servo-rotational-scale'));
  });

  statusPoseTopic.subscribe((msg: { status: number }) => {
    lastStatusPose = ARM_STATUSES[msg.status];
    window.dispatchEvent(new Event('pose-status'));
  });

  statusTrajectoryTopic.subscribe((msg: { status: number }) => {
    lastStatusTrajectory = ARM_STATUSES[msg.status];
    window.dispatchEvent(new Event('trajectory-status'));
  });

  setLinearScaleTo(lastServoLinearScale);
  setRotationalScaleTo(lastServoRotationalScale);
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

function sendPoseRequest(id: number) {
  if (executePoseTopic) {
    executePoseTopic.publish({ pose_id: id });
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

window.addEventListener('keydown', (event) => {
  // Check if any input box is focused.
  if (document.activeElement.tagName === 'INPUT') {
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
  }
});

export {
  setLinearScaleTo,
  setRotationalScaleTo,
  lastServoLinearScale,
  lastServoRotationalScale,
  abortPose,
  sendPoseRequest,
  keepAlivePose,
  lastStatusPose,
  abortTrajectory,
  sendTrajectoryRequest,
  keepAliveTrajectory,
  lastStatusTrajectory
};
