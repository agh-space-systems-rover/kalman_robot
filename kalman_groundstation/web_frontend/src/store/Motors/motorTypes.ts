export interface Wheel {
  velocity: number
  angle: number
}

export interface WheelScaling {
  forward: number
  turn: number
}

export interface Wheels {
  targetMotors: {
    fr: Wheel
    br: Wheel
    bl: Wheel
    fl: Wheel
  }
  motors: {
    fr: Wheel
    br: Wheel
    bl: Wheel
    fl: Wheel
  }
}

export interface MotorsTemperature {
  fr: {
    turn: number
    motor: number
    overheating: boolean
  }
  fl: {
    turn: number
    motor: number
    overheating: boolean
  }
  br: {
    turn: number
    motor: number
    overheating: boolean
  }
  bl: {
    turn: number
    motor: number
    overheating: boolean
  }
}

export enum DrivingMode {
  Normal = 'Normal 🙂',
  Sideways = 'Sideways 🤯',
  InPlace = 'InPlace 😵‍💫',
  // eslint-disable-next-line
  Digging = 'Diggin\' ⛏️',
}

export interface MotorsState {
  settings: MotorSettings
  wheels: Wheels
  temperature: MotorsTemperature | null
}

export interface MotorSettings {
  drivingMode: DrivingMode
  speed: number
  scalingFactor: WheelScaling
}
