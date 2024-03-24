export interface JointPositions {
  joints: number[]
  gripper: number
  status: string
  collisionVelocityFactor: number
}

export interface JointLimit {
  lower: number
  upper: number
}

export enum ArmMode {
  IK = 'InverseKinematics',
  FK = 'ForwardKinematics',
}

export enum KeyboardTypingMode {
  IK = 0,
  FK = 1,
  GRIPPER_CAN = 2,
  GRIPPER_HTTP = 3,
}

export interface ArmState {
  jointPositions: JointPositions
  jointLimits: JointLimit[]
  armMode: ArmMode
  keyboardTypingMode: KeyboardTypingMode
}

export const initialArmState: ArmState = {
  jointPositions: {
    joints: [0, 0, 0, 0, 0, 0],
    gripper: 0,
    status: 'unknown',
    collisionVelocityFactor: 0,
  },
  jointLimits: [
    { lower: -6.2815926, upper: 6.2815926 },
    { lower: -3.1415926, upper: 3.1415926 },
    { lower: -2.88, upper: 2.88 },
    { lower: -3.927, upper: 3.927 },
    { lower: -1.57, upper: 1.57 },
    { lower: -3.4032, upper: 3.4032 },
  ],
  armMode: ArmMode.FK,
  keyboardTypingMode: KeyboardTypingMode.IK,
}

export interface wsJointPositions {
  gripper_moving_joint: number
  joint_1: number
  joint_2: number
  joint_3: number
  joint_4: number
  joint_5: number
  joint_6: number
  servo_status: 'ServoStatus.OK'
  collisionVelocityFactor: number
}

export const convertJointPositions = (ws: wsJointPositions): JointPositions => {
  return {
    joints: [ws.joint_1, ws.joint_2, ws.joint_3, ws.joint_4, ws.joint_5, ws.joint_6],
    gripper: ws.gripper_moving_joint,
    status: ws.servo_status,
    collisionVelocityFactor: ws.collisionVelocityFactor,
  }
}
