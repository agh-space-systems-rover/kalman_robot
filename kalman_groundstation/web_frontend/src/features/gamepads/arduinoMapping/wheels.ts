import type { Action } from '@reduxjs/toolkit'

import { sendMessage } from '../../../components/Websocket/websocketSlice'
import { changeDrivingMode } from '../../../store/Motors/motorsSlice'
import { DrivingMode } from '../../../store/Motors/motorTypes'
import type { GamepadChange, WheelsCommand } from '../gamepadTypes'
import { Buttons } from './arduinoMapping'

export const wheelsMapping: GamepadChange = (args) => {
  if (args.wheels.drivingMode === DrivingMode.Digging) {
    return diggingMapping(args)
  } else {
    return drivingMapping(args)
  }
}

const diggingMapping: GamepadChange = (args) => {
  const { currentValues, previousValues } = args

  currentValues.axes = [
    currentValues.axes[4],
    currentValues.axes[1],
    currentValues.axes[0],
    currentValues.axes[2],
    currentValues.axes[5],
    currentValues.axes[3],
  ]

  const pressed = (button: Buttons): boolean => {
    if (!previousValues) return false
    return previousValues.buttons[button.valueOf()] - currentValues.buttons[button.valueOf()] === 1
  }

  if (pressed(Buttons.RB)) {
    return [changeDrivingMode(DrivingMode.Normal)]
  }

  const x = -currentValues.axes[1] * ((currentValues.axes[3] + 1) / 2)
  const y = -currentValues.axes[0] * ((currentValues.axes[4] + 1) / 2)

  // sends message to different topic than other modes
  return [sendMessage({ topic: '/station/digging/command', data: { data: [x, y] } })]
}

const drivingMapping: GamepadChange = (args) => {
  const { wheels, currentValues, previousValues } = args

  currentValues.axes = [
    currentValues.axes[4],
    currentValues.axes[1],
    currentValues.axes[0],
    currentValues.axes[2],
    currentValues.axes[5],
    currentValues.axes[3],
  ]

  const drivingMode = wheels.drivingMode
  const actions: Action<unknown>[] = []

  let message: WheelsCommand
  if (drivingMode == DrivingMode.Normal) {
    message = {
      mode: 0,
      x: -currentValues.axes[1] * ((currentValues.axes[3] + 1) / 2),
      y: -currentValues.axes[0] * ((currentValues.axes[4] + 1) / 2),
      z: 2.0 * -currentValues.axes[2] * ((currentValues.axes[5] + 1) / 2),
    }
  } else if (drivingMode == DrivingMode.InPlace) {
    message = {
      mode: 1,
      x: currentValues.axes[1] * ((currentValues.axes[3] + 1) / 2),
      y: 2.0 * -currentValues.axes[2] * ((currentValues.axes[4] + 1) / 2),
      z: currentValues.axes[2] * ((currentValues.axes[3] + 1) / 2),
    }
  } else {
    message = {
      mode: 2,
      x: -currentValues.axes[0] * ((currentValues.axes[3] + 1) / 2),
      y: -currentValues.axes[2], // skosy podczas sideways
      z: currentValues.axes[1], // skręcanie podczas sideways
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
  }
  return actions
}
