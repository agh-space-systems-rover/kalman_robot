import { ros } from './ros';
import { Action, Topic } from 'roslib';

export const ARM_AUTONOMY_TOPICS = {
  image: '/arm/panel/image_rectified',
  detections: '/arm/panel/detections_rectified',
  homography: '/arm/panel/homography',
  target: '/arm/panel/target',
  rockImage: '/d455_arm_wheel/yolo_annotated/compressed',
  rockDetections: '/arm/rock_detections',
  rockPositions: '/arm/rock_positions',
  compactAction: '/arm/goto_pose',
  rockTargetAction: '/arm/pose_ik_navigate_to_pose'
} as const;

const COMPACT_HERMAN_JOINTS = [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0] as const;
const ROCK_BASE_FRAME = 'base_link';
const ROCK_END_EFFECTOR_FRAME = 'arm_link_end';
const ROCK_APPROACH_TIMEOUT_SECONDS = 15;

export type ImageXY = { x: number; y: number };
export type SendXY = { x: number; y: number };

export interface Header {
  stamp?: { sec?: number; nanosec?: number };
  frame_id?: string;
}

export interface RosImage {
  height: number;
  width: number;
  encoding: string;
  step: number;
  data: string | number[];
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

export interface ArmAutonomyTargetPayload {
  imageXY: ImageXY;
  sendXY: SendXY;
  label?: string;
}

const UPDATE_EVENT = 'arm-autonomy-update';
const LOG_PREFIX = '[arm-autonomy]';

let latestImage: RosImage | null = null;
let latestDetections: Detection2D[] = [];
let latestHomography: number[] | null = null;
let latestRockImage: RosCompressedImage | null = null;
let latestRockDetections: Detection2D[] = [];
let latestRockPositions: PoseStamped['pose'][] = [];
let latestRockFrame = 'base_link';
let topicsInitialized = false;
let imageFrameCount = 0;
let targetTopic: Topic<PoseStamped> | null = null;
let compactActionClient: Action<ArmGotoJointPoseGoal, ArmGotoJointPoseFeedback, Record<string, never>> | null = null;
let rockTargetActionClient: Action<
  PoseIKNavigateToPoseGoal,
  PoseIKNavigateToPoseFeedback,
  PoseIKNavigateToPoseResult
> | null = null;
let rockActionSequence = 0;
let currentRockAction: {
  client:
    | Action<ArmGotoJointPoseGoal, ArmGotoJointPoseFeedback, Record<string, never>>
    | Action<PoseIKNavigateToPoseGoal, PoseIKNavigateToPoseFeedback, PoseIKNavigateToPoseResult>;
  id: string;
} | null = null;

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

export function getArmAutonomyImage(): RosImage | null {
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

function getTargetPublisher(): Topic<PoseStamped> {
  if (!targetTopic) {
    targetTopic = new Topic<PoseStamped>({
      ros,
      name: ARM_AUTONOMY_TOPICS.target,
      messageType: 'geometry_msgs/PoseStamped'
    });
  }
  return targetTopic;
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
  log('sent compact_herman action', compactGoal);
  return true;
}

export function stopArmAutonomyRockTarget(): void {
  ++rockActionSequence;
  if (currentRockAction) {
    currentRockAction.client.cancelGoal(currentRockAction.id);
    currentRockAction = null;
  }
}

export function publishArmAutonomyTarget(payload: ArmAutonomyTargetPayload): boolean {
  if (!ros.isConnected) {
    console.warn(`${LOG_PREFIX} cannot publish target — ROS disconnected`);
    return false;
  }

  ensureArmAutonomySubscriptions();

  const nowMs = Date.now();
  const msg: PoseStamped = {
    header: {
      stamp: {
        sec: Math.floor(nowMs / 1000),
        nanosec: (nowMs % 1000) * 1_000_000
      },
      frame_id: payload.label ?? 'panel'
    },
    pose: {
      position: {
        x: payload.sendXY.x,
        y: payload.sendXY.y,
        z: 0
      },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    }
  };

  getTargetPublisher().publish(msg);
  log('published panel target', {
    topic: ARM_AUTONOMY_TOPICS.target,
    label: msg.header.frame_id,
    imageXY: payload.imageXY,
    sendXY: payload.sendXY,
    message: msg
  });
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
