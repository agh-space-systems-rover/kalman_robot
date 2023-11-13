import type { Action } from '@reduxjs/toolkit'

import { sendMessage } from '../../../components/Websocket/websocketSlice'
import { decreaseCamera, increaseCamera } from '../../../store/Feeds/feedsSlice'
import { changeDrivingMode, changeWheelsScalingFactor } from '../../../store/Motors/motorsSlice'
import type { WheelScaling } from '../../../store/Motors/motorTypes'
import { DrivingMode } from '../../../store/Motors/motorTypes'
import type { GamepadChange, WheelsCommand } from '../gamepadTypes'
import { Buttons } from './standardMapping'

export const scalingPresets: WheelScaling[] = [
  { forward: 1, turn: 1 },
  { forward: 0.5, turn: 0.5 },
  { forward: 0.3, turn: 0.3 },
  { forward: 0.15, turn: 0.15 },
  { forward: 1, turn: 0.3 },
  { forward: 1, turn: 0.6 },
  { forward: 1, turn: 1 },
  // { forward: 0.9, turn: 1 },
  { forward: 0.8, turn: 1 },
  // { forward: 0.7, turn: 1 },
  { forward: 0.6, turn: 1 },
  // { forward: 0.5, turn: 1 },
  { forward: 0.4, turn: 1 },
  // { forward: 0.3, turn: 1 },
  { forward: 0.2, turn: 1 },
  // { forward: 0.1, turn: 1 },
  { forward: 0.0, turn: 1 },
]

export const wheelsMapping: GamepadChange = (args) => {
  if (args.wheels.drivingMode === DrivingMode.Digging) {
    return diggingMapping(args)
  } else {
    // other driving modes
    return drivingMapping(args)
  }
}

const diggingMapping: GamepadChange = (args) => {
  const { currentValues, previousValues } = args

  // map input from triggers to values from range [-1, 1]
  const x = (-1 - currentValues.axes[2]) / 2 - (-1 - currentValues.axes[5]) / 2
  const y = -currentValues.axes[0]
  const message = { data: [x, y] }

  const pressed = (button: Buttons): boolean => {
    if (!previousValues) return false
    return previousValues.buttons[button.valueOf()] - currentValues.buttons[button.valueOf()] === 1
  }

  if (pressed(Buttons.RB)) {
    return [changeDrivingMode(DrivingMode.Normal)]
  }

  // sends message to different topic than other modes
  return [sendMessage({ topic: '/station/digging/command', data: message })]
}

const drivingMapping: GamepadChange = (args) => {
  const { wheels, currentValues, previousValues } = args

  const scalingFactor = wheels.scalingFactor
  const drivingMode = wheels.drivingMode
  const actions: Action<unknown>[] = []

  // map input from triggers to values from range [-1, 1]
  const x = (-1 - currentValues.axes[2]) / 2 - (-1 - currentValues.axes[5]) / 2

  let message: WheelsCommand
  if (drivingMode == DrivingMode.Normal) {
    message = {
      mode: 0,
      x: x * scalingFactor.forward,
      y: -currentValues.axes[0] * scalingFactor.turn,
      z: 2.0 * -currentValues.axes[3],
    }
  } else if (drivingMode == DrivingMode.InPlace) {
    message = {
      mode: 1,
      x: x,
      y: 2.0 * -currentValues.axes[3] * scalingFactor.turn,
      z: currentValues.axes[0] * scalingFactor.turn,
    }
  } else {
    message = {
      mode: 2,
      x: -x * scalingFactor.forward,
      y: (x < 0 ? 1 : -1) * currentValues.axes[4],
      z: -currentValues.axes[1],
    }
  }

  actions.push(sendMessage({ topic: '/station/wheels/command', data: message }))

  // detect button after second iteration
  if (previousValues) {
    const pressed = (button: Buttons): boolean => {
      return previousValues.buttons[button.valueOf()] - currentValues.buttons[button.valueOf()] === 1
    }
    const holding = (button: Buttons): boolean => {
      return previousValues.buttons[button.valueOf()] === 1
    }

    // speed scaling
    if (pressed(Buttons.A)) {
      const index = scalingPresets.indexOf(wheels.scalingFactor)
      actions.push(changeWheelsScalingFactor(scalingPresets[Math.min(scalingPresets.length - 1, index + 1)]))
    }
    if (pressed(Buttons.Y)) {
      const index = scalingPresets.indexOf(wheels.scalingFactor)
      actions.push(changeWheelsScalingFactor(scalingPresets[Math.max(0, index - 1)]))
    }

    // driving mode
    if (pressed(Buttons.RB) || pressed(Buttons.LB)) {
      let newDrivingMode: DrivingMode = DrivingMode.Normal

      if (pressed(Buttons.RB) && drivingMode !== DrivingMode.InPlace && !holding(Buttons.LB)) {
        newDrivingMode = DrivingMode.InPlace
      }
      if (pressed(Buttons.LB) && drivingMode !== DrivingMode.Sideways && !holding(Buttons.RB)) {
        newDrivingMode = DrivingMode.Sideways
      }
      actions.push(changeDrivingMode(newDrivingMode))
    }

    // video feed control
    if (pressed(Buttons.X)) {
      actions.push(decreaseCamera(0))
    }

    if (pressed(Buttons.B)) {
      actions.push(increaseCamera(0))
    }
  }
  return actions
}
