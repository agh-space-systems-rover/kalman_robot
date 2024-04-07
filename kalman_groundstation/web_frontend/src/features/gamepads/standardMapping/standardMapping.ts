import type { Action } from 'redux'

import { pressedTrigger } from '../../../store/Gamepads/gamepadSlice'
import type { GamepadChange, MappingResult } from '../gamepadTypes'
import { Side, Target } from '../gamepadTypes'
import { armMapping } from './arm'
import { drillMapping } from './drill'
import { wheelsMapping } from './wheels'

export enum Buttons {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  RB = 5,
  LB = 4,
  LOGI = 16,
  BACK = 8,
  START = 9,
  L3 = 10, // Analog stick click
  R3 = 11, // Analog stick click
  DPAD_R = 15,
  DPAD_L = 14,
  DPAD_UP = 12,
  DPAD_DOWN = 13,
}

export enum Axis {
  LT = 4,
  RT = 5,
  L3_HORIZONTAL = 0,
  L3_VERTICAL = 1,
  R3_HORIZONTAL = 2,
  R3_VERTICAL = 3,
}

// Mapping for XBOX gamepads.

export const standardMapping: GamepadChange = (args) => {
  const { config, currentValues } = args

  // Gamepads that we used had this driver issue where they would display correct values for triggers only
  // after pressing them at least once. Here we ignore those values until we know  they are correct
  const triggerChanges: Action<unknown>[] = []
  if (!config.leftTriggerPressed && currentValues.axes[Axis.LT] == -1) {
    triggerChanges.push(pressedTrigger({ index: config.index, side: Side.Left }))
  }
  if (!config.rightTriggerPressed && currentValues.axes[Axis.RT] == -1) {
    triggerChanges.push(pressedTrigger({ index: config.index, side: Side.Right }))
  }
  if (!config.leftTriggerPressed || !config.rightTriggerPressed) {
    currentValues.axes[Axis.LT] = -1
    currentValues.axes[Axis.RT] = -1
  }

  let response: MappingResult
  if (config.target === Target.Wheels) {
    response = wheelsMapping(args)
  } else if (config.target === Target.Arm) {
    response = armMapping(args)
  } else {
    response = drillMapping(args)
  }
  return response.concat(triggerChanges)
}
