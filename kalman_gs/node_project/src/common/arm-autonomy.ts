import { ros } from './ros';
import { Action, Topic } from 'roslib';

export const ARM_AUTONOMY_TOPICS = {
  image: '/arm/panel/image_rectified',
  detections: '/arm/panel/detections_rectified',
  homography: '/arm/panel/homography'
} as const;

export const ARM_AUTONOMY_ACTIONS = {
  moveToPanelPose: '/arm/move_to_panel_pose',
  calibratePanelMarkerIds: '/arm/calibrate_panel_marker_ids'
} as const;

const TARGET_Z_METERS = 0.05;

export type ImageXY = { x: number; y: number };
export type SendXY = { x: number; y: number };

export interface RosImage {
  height: number;
  width: number;
  encoding: string;
  step: number;
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

let latestImage: RosImage | null = null;
let latestDetections: Detection2D[] = [];
let latestHomography: number[] | null = null;
let topicsInitialized = false;
let imageFrameCount = 0;
let moveToPanelPoseAction: Action<
  MoveToPanelPoseGoal,
  MoveToPanelPoseFeedback,
  MoveToPanelPoseResult
> | null = null;
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

function imageDataToBytes(data: string | number[]): Uint8Array {
  if (typeof data === 'string') {
    const binary = atob(data);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) {
      bytes[i] = binary.charCodeAt(i);
    }
    return bytes;
  }
  return Uint8Array.from(data);
}

export function decodeRosImage(msg: RosImage): ImageData | null {
  const { width, height, encoding, step } = msg;
  if (width <= 0 || height <= 0) {
    return null;
  }

  const bytes = imageDataToBytes(msg.data);
  const rgba = new Uint8ClampedArray(width * height * 4);
  const enc = encoding.toLowerCase();

  if (enc === 'rgb8') {
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const src = row + x * 3;
        const dst = (y * width + x) * 4;
        rgba[dst] = bytes[src];
        rgba[dst + 1] = bytes[src + 1];
        rgba[dst + 2] = bytes[src + 2];
        rgba[dst + 3] = 255;
      }
    }
  } else if (enc === 'bgr8') {
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const src = row + x * 3;
        const dst = (y * width + x) * 4;
        rgba[dst] = bytes[src + 2];
        rgba[dst + 1] = bytes[src + 1];
        rgba[dst + 2] = bytes[src];
        rgba[dst + 3] = 255;
      }
    }
  } else if (enc === 'mono8') {
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const gray = bytes[row + x];
        const dst = (y * width + x) * 4;
        rgba[dst] = gray;
        rgba[dst + 1] = gray;
        rgba[dst + 2] = gray;
        rgba[dst + 3] = 255;
      }
    }
  } else {
    console.warn(`arm-autonomy: unsupported image encoding "${encoding}"`);
    return null;
  }

  return new ImageData(rgba, width, height);
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

export function getArmAutonomyImage(): RosImage | null {
  return latestImage;
}

export function getArmAutonomyDetections(): Detection2D[] {
  return latestDetections;
}

export function getArmAutonomyHomography(): number[] | null {
  return latestHomography;
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

  const imageTopic = new Topic<RosImage>({
    ros,
    name: ARM_AUTONOMY_TOPICS.image,
    messageType: 'sensor_msgs/Image'
  });
  imageTopic.subscribe((msg) => {
    latestImage = msg;
    imageFrameCount += 1;
    if (imageFrameCount === 1 || imageFrameCount % 30 === 0) {
      log(`image frame #${imageFrameCount}`, {
        width: msg.width,
        height: msg.height,
        encoding: msg.encoding,
        step: msg.step,
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
}

window.addEventListener('ros-connect', initTopics);
