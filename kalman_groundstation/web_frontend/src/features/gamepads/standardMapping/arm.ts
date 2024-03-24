import type { Action } from '@reduxjs/toolkit'

import { sendMessage } from '../../../components/Websocket/websocketSlice'
// import { changeArmMode } from '../../../store/Arm/armSlice'
import { ArmMode } from '../../../store/Arm/armTypes'
import { decreaseCamera, increaseCamera } from '../../../store/Feeds/feedsSlice'
import type { GamepadChange } from '../gamepadTypes'
import { Buttons } from './standardMapping'

export interface ArmFkCommand {
  gripper: number
  joint_1: number
  joint_2: number
  joint_3: number
  joint_4: number
  joint_5: number
  joint_6: number
}

export interface ArmIkCommand {
  linear_x: number
  linear_y: number
  linear_z: number
  angular_x: number
  angular_y: number
  angular_z: number
}

export const armMapping: GamepadChange = (args) => {
  const { armMode, currentValues, previousValues } = args

  const actions: Action<unknown>[] = []

  if (previousValues) {
    const pressed = (button: Buttons): boolean => {
      return previousValues.buttons[button.valueOf()] - currentValues.buttons[button.valueOf()] === -1
    }

    // if (pressed(Buttons.LB)) {
    //   console.log('Switched to using Inverse Kinematics')
    //   actions.push(changeArmMode(ArmMode.IK))
    // }

    // if (pressed(Buttons.RB)) {
    //   console.log('Switched to using Forward Kinematics')
    //   actions.push(changeArmMode(ArmMode.FK))
    // }

    // video feed control
    if (pressed(Buttons.X)) {
      actions.push(decreaseCamera(1))
    }

    if (pressed(Buttons.Y)) {
      actions.push(increaseCamera(1))
    }
  }

  if (armMode === ArmMode.IK) {
    // Inverse Kinematics
    const messageIK: ArmIkCommand = {
      // eslint-disable-next-line camelcase
      linear_x: currentValues.axes[4],
      // eslint-disable-next-line camelcase
      linear_y: currentValues.axes[3],
      // eslint-disable-next-line camelcase
      linear_z: (-1 - currentValues.axes[2]) / 2 - (-1 - currentValues.axes[5]) / 2,
      // eslint-disable-next-line camelcase
      angular_x: currentValues.axes[0],
      // eslint-disable-next-line camelcase
      angular_y: currentValues.axes[1],
      // eslint-disable-next-line camelcase
      angular_z: currentValues.axes[6],
    }
    actions.push(sendMessage({ topic: '/station/arm/ik/command', data: messageIK }))
  }

  if (armMode === ArmMode.FK) {
    // Forward Kinematics
    const messageFK = {
      gripper: currentValues.buttons[Buttons.A] - currentValues.buttons[Buttons.B],
      // eslint-disable-next-line camelcase
      joint_1: -currentValues.axes[0],
      // eslint-disable-next-line camelcase
      joint_2: -currentValues.axes[1],
      // eslint-disable-next-line camelcase
      joint_3: -currentValues.axes[4],
      // eslint-disable-next-line camelcase
      joint_4: currentValues.axes[3],
      // eslint-disable-next-line camelcase
      joint_5: -currentValues.axes[6],
      // eslint-disable-next-line camelcase
      joint_6: (-1 - currentValues.axes[2]) / 2 - (-1 - currentValues.axes[5]) / 2,
    }

    actions.push(sendMessage({ topic: '/station/arm/fk/command', data: messageFK }))
  }

  return actions
}
