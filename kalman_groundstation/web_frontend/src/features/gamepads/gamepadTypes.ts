import type { Action } from 'redux'

import type { ArmMode } from '../../store/Arm/armTypes'
import type { gamepadSettings } from '../../store/Keybinds/keybindsTypes'
import type { MotorSettings } from '../../store/Motors/motorTypes'

export interface WheelsCommand {
  mode: number
  x: number
  y: number
  z: number
}

export enum Target {
  Wheels = 'Wheels',
  Arm = 'Arm',
  Drill = 'Drill',
  None = 'Disabled',
}

export enum Speed {
  Normal = 'Normal',
  Reduced = 'Reduced',
}

export interface GamepadConfig {
  index: number
  name: string
  target: Target
  leftTriggerPressed: boolean
  rightTriggerPressed: boolean
}

export enum Side {
  Left,
  Right,
}
export interface GamepadValues {
  buttons: number[]
  axes: number[]
}

export interface MappingArguments {
  config: GamepadConfig
  wheels: MotorSettings
  armMode: ArmMode
  currentValues: GamepadValues
  previousValues: GamepadValues | undefined
  gamepadBinds: gamepadSettings
}

export type MappingResult = Action<unknown>[]

export interface GamepadChange {
  (args: MappingArguments): MappingResult
}
