import { ros } from './ros';
import { Action, Topic } from 'roslib';

export const ARM_AUTONOMY_TOPICS = {
  image: '/arm/panel/image_rectified/compressed',
  detections: '/arm/panel/detections_rectified',
  homography: '/arm/panel/homography',
  rockImage: '/d455_arm_wheel/yolo_annotated/compressed',
  rockDetections: '/arm/rock_detections',
  rockPositions: '/arm/rock_positions',
  jawTarget: '/arm/target_pos/jaw',
  compactAction: '/arm/goto_pose',
  rockTargetAction: '/arm/pose_ik_navigate_to_pose'
} as const;

export const ARM_AUTONOMY_ACTIONS = {
  moveToPanelPose: '/arm/move_to_panel_pose',
  calibratePanelMarkerIds: '/arm/calibrate_panel_marker_ids'
} as const;

const TARGET_Z_METERS = 0.05;
const COMPACT_HERMAN_JOINTS = [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0] as const;
const JAW_CLOSED_POSITION = 0.0;
const JAW_OPEN_POSITION = 1.57;
const ROCK_BASE_FRAME = 'base_link';
const ROCK_END_EFFECTOR_FRAME = 'arm_link_end';
const ROCK_APPROACH_TIMEOUT_SECONDS = 15;

export type ImageXY = { x: number; y: number };
export type SendXY = { x: number; y: number };

export interface Header {
  stamp?: { sec?: number; nanosec?: number };
  frame_id?: string;
}

export interface PoseStamped {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}


export interface RosCompressedImage {
  header?: Header;
  format: string;
  data: string | number[];
}

export interface ObjectHypothesis {
  class_id: string;
  score: number;
}

export interface ObjectHypothesisWithPose {
  hypothesis: ObjectHypothesis;
}

export interface BoundingBox2D {
  center: { position: { x: number; y: number } };
  size_x: number;
  size_y: number;
}

export interface Detection2D {
  id?: string;
  header?: Header;
  bbox: BoundingBox2D;
  results: ObjectHypothesisWithPose[];
}

export interface Detection2DArray {
  detections: Detection2D[];
}

export interface Float64MultiArray {
  data: number[];
}

export interface MoveToPanelPoseGoal {
  target_pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  behavior_tree: string;
}

export interface PoseArray {
  header?: Header;
  poses: PoseStamped['pose'][];
}

interface ArmValues {
  header: Header;
  joints: number[];
  jaw: number;
}

interface ArmGotoJointPoseGoal {
  target_pos: ArmValues;
  ignore_mask: number;
}

interface ArmGotoJointPoseFeedback {
  current_pos: ArmValues;
}

interface PoseIKNavigateToPoseGoal {
  pose: PoseStamped['pose'];
  base_frame: string;
  end_effector_link: string;
  timeout_seconds: number;
}

interface PoseIKNavigateToPoseResult {
  result: boolean;
  message: string;
}

interface PoseIKNavigateToPoseFeedback {
  position_error: number;
  orientation_error: number;
}

export interface MoveToPanelPoseFeedback {
  progress: string;
}

export interface MoveToPanelPoseResult {
  result: boolean;
  message: string;
}

export interface ArmAutonomyGoalCallbacks {
  onResult: (result: MoveToPanelPoseResult) => void;
  onFeedback: (feedback: MoveToPanelPoseFeedback) => void;
  onFailed: (error: string) => void;
}

export interface CalibratePanelMarkerIdsGoal {
  required_confirmations: number;
  timeout_seconds: number;
}

export interface CalibratePanelMarkerIdsFeedback {
  confirmations: number;
  progress: string;
}

export interface CalibratePanelMarkerIdsResult {
  result: boolean;
  message: string;
  marker_names: string[];
  marker_ids: number[];
}

export interface PanelMarkerCalibrationCallbacks {
  onResult: (result: CalibratePanelMarkerIdsResult) => void;
  onFeedback: (feedback: CalibratePanelMarkerIdsFeedback) => void;
  onFailed: (error: string) => void;
}

const UPDATE_EVENT = 'arm-autonomy-update';
const LOG_PREFIX = '[arm-autonomy]';

let latestImage: RosCompressedImage | null = null;
let latestDetections: Detection2D[] = [];
let latestHomography: number[] | null = null;
let latestRockImage: RosCompressedImage | null = null;
let latestRockDetections: Detection2D[] = [];
let latestRockPositions: PoseStamped['pose'][] = [];
let latestRockFrame = 'base_link';
let topicsInitialized = false;
let imageFrameCount = 0;
let jawTargetTopic: Topic<ArmValues> | null = null;
let compactActionClient: Action<ArmGotoJointPoseGoal, ArmGotoJointPoseFeedback, Record<string, never>> | null = null;
let rockTargetActionClient: Action<
  PoseIKNavigateToPoseGoal,
  PoseIKNavigateToPoseFeedback,
  PoseIKNavigateToPoseResult
> | null = null;
let moveToPanelPoseAction: Action<
  MoveToPanelPoseGoal,
  MoveToPanelPoseFeedback,
  MoveToPanelPoseResult
> | null = null;
let rockActionSequence = 0;
let currentRockAction: {
  client:
    | Action<ArmGotoJointPoseGoal, ArmGotoJointPoseFeedback, Record<string, never>>
    | Action<PoseIKNavigateToPoseGoal, PoseIKNavigateToPoseFeedback, PoseIKNavigateToPoseResult>;
  id: string;
} | null = null;
let calibratePanelMarkerIdsAction: Action<
  CalibratePanelMarkerIdsGoal,
  CalibratePanelMarkerIdsFeedback,
  CalibratePanelMarkerIdsResult
> | null = null;

function log(message: string, data?: unknown) {
  if (data === undefined) {
    console.log(`${LOG_PREFIX} ${message}`);
  } else {
    console.log(`${LOG_PREFIX} ${message}`, data);
  }
}

function notifyUpdate() {
  window.dispatchEvent(new CustomEvent(UPDATE_EVENT));
}


export function compressedImageDataUrl(msg: RosCompressedImage | null): string | null {
  if (!msg) {
    return null;
  }
  let base64: string;
  if (typeof msg.data === 'string') {
    base64 = msg.data;
  } else {
    const bytes = Uint8Array.from(msg.data);
    let binary = '';
    const chunkSize = 0x8000;
    for (let offset = 0; offset < bytes.length; offset += chunkSize) {
      binary += String.fromCharCode(...bytes.subarray(offset, offset + chunkSize));
    }
    base64 = btoa(binary);
  }
  const mime = msg.format.toLowerCase().includes('png') ? 'image/png' : 'image/jpeg';
  return `data:${mime};base64,${base64}`;
}

export function applyHomography(H: number[], imageX: number, imageY: number): SendXY | null {
  if (H.length < 9) {
    return null;
  }
  const xp = H[0] * imageX + H[1] * imageY + H[2];
  const yp = H[3] * imageX + H[4] * imageY + H[5];
  const w = H[6] * imageX + H[7] * imageY + H[8];
  if (Math.abs(w) < 1e-9) {
    return null;
  }
  return { x: xp / w, y: yp / w };
}

export function detectionCenter(det: Detection2D): ImageXY {
  return {
    x: det.bbox.center.position.x,
    y: det.bbox.center.position.y
  };
}

export function detectionLabel(det: Detection2D): string {
  const result = det.results[0];
  if (!result) {
    return 'detection';
  }
  const score = (result.hypothesis.score * 100).toFixed(0);
  return `${result.hypothesis.class_id} (${score}%)`;
}

export function getArmAutonomyImage(): RosCompressedImage | null {
  return latestImage;
}

export function getArmAutonomyDetections(): Detection2D[] {
  return latestDetections;
}

export function getArmAutonomyHomography(): number[] | null {
  return latestHomography;
}

export function getArmAutonomyRockImage(): RosCompressedImage | null {
  return latestRockImage;
}

export function getArmAutonomyRockDetections(): Detection2D[] {
  return latestRockDetections;
}

export function getArmAutonomyRockPositions(): PoseStamped['pose'][] {
  return latestRockPositions;
}

export function addArmAutonomyListener(listener: () => void): () => void {
  const handler = () => listener();
  window.addEventListener(UPDATE_EVENT, handler);
  return () => window.removeEventListener(UPDATE_EVENT, handler);
}

export function ensureArmAutonomySubscriptions(): void {
  if (ros.isConnected) {
    initTopics();
  }
}

function getMoveToPanelPoseAction(): Action<
  MoveToPanelPoseGoal,
  MoveToPanelPoseFeedback,
  MoveToPanelPoseResult
> {
  if (!moveToPanelPoseAction) {
    moveToPanelPoseAction = new Action({
      ros,
      name: ARM_AUTONOMY_ACTIONS.moveToPanelPose,
      actionType: 'kalman_interfaces/MoveToPanelPose'
    });
  }
  return moveToPanelPoseAction;
}

function getCompactActionClient(): Action<ArmGotoJointPoseGoal, ArmGotoJointPoseFeedback, Record<string, never>> {
  if (!compactActionClient) {
    compactActionClient = new Action({
      ros,
      name: ARM_AUTONOMY_TOPICS.compactAction,
      actionType: 'kalman_interfaces/ArmGotoJointPose'
    });
  }
  return compactActionClient;
}

function publishJawTarget(jaw: number): void {
  if (!jawTargetTopic) {
    jawTargetTopic = new Topic<ArmValues>({
      ros,
      name: ARM_AUTONOMY_TOPICS.jawTarget,
      messageType: 'kalman_interfaces/ArmValues'
    });
  }
  const nowMs = Date.now();
  jawTargetTopic.publish({
    header: {
      stamp: {
        sec: Math.floor(nowMs / 1000),
        nanosec: (nowMs % 1000) * 1_000_000
      },
      frame_id: ''
    },
    joints: [0, 0, 0, 0, 0, 0],
    jaw
  });
}

function getRockTargetActionClient(): Action<
  PoseIKNavigateToPoseGoal,
  PoseIKNavigateToPoseFeedback,
  PoseIKNavigateToPoseResult
> {
  if (!rockTargetActionClient) {
    rockTargetActionClient = new Action({
      ros,
      name: ARM_AUTONOMY_TOPICS.rockTargetAction,
      actionType: 'kalman_interfaces/PoseIKNavigateToPose'
    });
  }
  return rockTargetActionClient;
}

export function publishArmAutonomyRockTarget(pose: PoseStamped['pose']): boolean {
  if (
    !ros.isConnected ||
    latestRockFrame !== ROCK_BASE_FRAME ||
    !Number.isFinite(pose.position.x) ||
    !Number.isFinite(pose.position.y) ||
    !Number.isFinite(pose.position.z) ||
    Math.hypot(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) <= 0.5
  ) {
    return false;
  }

  stopArmAutonomyRockTarget();
  const sequence = ++rockActionSequence;
  publishJawTarget(JAW_CLOSED_POSITION);
  const nowMs = Date.now();
  const compactGoal: ArmGotoJointPoseGoal = {
    target_pos: {
      header: {
        stamp: {
          sec: Math.floor(nowMs / 1000),
          nanosec: (nowMs % 1000) * 1_000_000
        },
        frame_id: ''
      },
      joints: [...COMPACT_HERMAN_JOINTS],
      jaw: 0
    },
    // compact_herman specifies joints 1-6 only, so preserve the jaw.
    ignore_mask: 1 << 6
  };
  const compactClient = getCompactActionClient();
  const compactGoalId = compactClient.sendGoal(
    compactGoal,
    () => {
      if (sequence !== rockActionSequence) {
        return;
      }
      const targetGoal: PoseIKNavigateToPoseGoal = {
        pose,
        base_frame: ROCK_BASE_FRAME,
        end_effector_link: ROCK_END_EFFECTOR_FRAME,
        timeout_seconds: ROCK_APPROACH_TIMEOUT_SECONDS
      };
      const targetClient = getRockTargetActionClient();
      const targetGoalId = targetClient.sendGoal(
        targetGoal,
        (result) => {
          if (sequence !== rockActionSequence) {
            return;
          }
          currentRockAction = null;
          if (result.result) {
            publishJawTarget(JAW_OPEN_POSITION);
            log('rock target action succeeded', result);
          } else {
            console.warn(`${LOG_PREFIX} rock target action failed`, result);
          }
        },
        undefined,
        (error) => {
          if (sequence !== rockActionSequence) {
            return;
          }
          currentRockAction = null;
          console.warn(`${LOG_PREFIX} rock target action failed`, error);
        }
      );
      if (!targetGoalId) {
        currentRockAction = null;
        console.warn(`${LOG_PREFIX} failed to send rock target action`);
        return;
      }
      currentRockAction = {
        client: targetClient,
        id: targetGoalId
      };
      log('compact_herman reached; sent rock target action', targetGoal);
    },
    undefined,
    (error) => {
      if (sequence !== rockActionSequence) {
        return;
      }
      currentRockAction = null;
      console.warn(`${LOG_PREFIX} compact_herman action failed`, error);
    }
  );
  if (!compactGoalId) {
    return false;
  }
  currentRockAction = {
    client: compactClient,
    id: compactGoalId
  };
  log('closed gripper; sent compact_herman action', compactGoal);
  return true;
}

export function stopArmAutonomyRockTarget(): void {
  ++rockActionSequence;
  if (currentRockAction) {
    currentRockAction.client.cancelGoal(currentRockAction.id);
    currentRockAction = null;
  }
}

export function sendArmAutonomyGoal(
  sendXY: SendXY,
  behaviorTree: string,
  callbacks: ArmAutonomyGoalCallbacks
): string | null {
  if (!ros.isConnected) {
    callbacks.onFailed('ROS disconnected');
    return null;
  }

  const goal: MoveToPanelPoseGoal = {
    target_pose: {
      position: { x: sendXY.x, y: sendXY.y, z: TARGET_Z_METERS },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    },
    behavior_tree: behaviorTree
  };

  const goalId = getMoveToPanelPoseAction().sendGoal(
    goal,
    callbacks.onResult,
    callbacks.onFeedback,
    callbacks.onFailed
  );
  log('sent MoveToPanelPose goal', {
    action: ARM_AUTONOMY_ACTIONS.moveToPanelPose,
    behaviorTree,
    sendXY,
    targetZ: TARGET_Z_METERS,
    goalId
  });
  return goalId ?? null;
}

export function inferPanelMarkerPositions(
  callbacks: PanelMarkerCalibrationCallbacks
): string | null {
  if (!ros.isConnected) {
    callbacks.onFailed('ROS disconnected');
    return null;
  }
  if (!calibratePanelMarkerIdsAction) {
    calibratePanelMarkerIdsAction = new Action({
      ros,
      name: ARM_AUTONOMY_ACTIONS.calibratePanelMarkerIds,
      actionType: 'kalman_interfaces/CalibratePanelMarkerIds'
    });
  }

  const goalId = calibratePanelMarkerIdsAction.sendGoal(
    { required_confirmations: 5, timeout_seconds: 10.0 },
    callbacks.onResult,
    callbacks.onFeedback,
    callbacks.onFailed
  );
  log('started panel marker ID inference', { goalId });
  return goalId ?? null;
}

export function cancelArmAutonomyGoal(goalId: string): boolean {
  if (!ros.isConnected || !moveToPanelPoseAction) {
    return false;
  }
  moveToPanelPoseAction.cancelGoal(goalId);
  log('requested MoveToPanelPose cancellation', { goalId });
  return true;
}

function initTopics() {
  if (topicsInitialized) {
    return;
  }
  topicsInitialized = true;
  log('subscribing to ROS topics', ARM_AUTONOMY_TOPICS);

  const imageTopic = new Topic<RosCompressedImage>({
    ros,
    name: ARM_AUTONOMY_TOPICS.image,
    messageType: 'sensor_msgs/CompressedImage'
  });
  imageTopic.subscribe((msg) => {
    latestImage = msg;
    imageFrameCount += 1;
    if (imageFrameCount === 1 || imageFrameCount % 30 === 0) {
      log(`compressed image frame #${imageFrameCount}`, {
        format: msg.format,
        dataType: typeof msg.data,
        dataLength: typeof msg.data === 'string' ? msg.data.length : msg.data?.length
      });
    }
    notifyUpdate();
  });

  const detectionsTopic = new Topic<Detection2DArray>({
    ros,
    name: ARM_AUTONOMY_TOPICS.detections,
    messageType: 'vision_msgs/Detection2DArray'
  });
  detectionsTopic.subscribe((msg) => {
    latestDetections = msg.detections ?? [];
    log(`rectified detections: ${latestDetections.length}`);
    notifyUpdate();
  });

  const homographyTopic = new Topic<Float64MultiArray>({
    ros,
    name: ARM_AUTONOMY_TOPICS.homography,
    messageType: 'std_msgs/Float64MultiArray'
  });
  homographyTopic.subscribe((msg) => {
    latestHomography = msg.data?.length >= 9 ? msg.data : null;
    if (latestHomography) {
      log('homography received', { length: msg.data.length, data: latestHomography.slice(0, 9) });
    } else {
      console.warn(`${LOG_PREFIX} homography too short`, { length: msg.data?.length ?? 0 });
    }
    notifyUpdate();
  });

  const rockImageTopic = new Topic<RosCompressedImage>({
    ros,
    name: ARM_AUTONOMY_TOPICS.rockImage,
    messageType: 'sensor_msgs/CompressedImage'
  });
  rockImageTopic.subscribe((msg) => {
    latestRockImage = msg;
    notifyUpdate();
  });

  const rockDetectionsTopic = new Topic<Detection2DArray>({
    ros,
    name: ARM_AUTONOMY_TOPICS.rockDetections,
    messageType: 'vision_msgs/Detection2DArray'
  });
  rockDetectionsTopic.subscribe((msg) => {
    latestRockDetections = msg.detections ?? [];
    notifyUpdate();
  });

  const rockPositionsTopic = new Topic<PoseArray>({
    ros,
    name: ARM_AUTONOMY_TOPICS.rockPositions,
    messageType: 'geometry_msgs/PoseArray'
  });
  rockPositionsTopic.subscribe((msg) => {
    latestRockPositions = msg.poses ?? [];
    latestRockFrame = msg.header?.frame_id || 'base_link';
    notifyUpdate();
  });
}

window.addEventListener('ros-connect', initTopics);
