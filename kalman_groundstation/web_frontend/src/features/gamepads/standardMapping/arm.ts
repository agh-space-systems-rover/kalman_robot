import type { Action } from '@reduxjs/toolkit'

import { sendMessage } from '../../../components/Websocket/websocketSlice'
// import { changeArmMode } from '../../../store/Arm/armSlice'
import { ArmMode } from '../../../store/Arm/armTypes'
import { decreaseCamera, increaseCamera } from '../../../store/Feeds/feedsSlice'
import { translateGamepadAxis, translateGamepadButton } from '../../../store/Keybinds/keybindsSlice'
import type { GamepadChange } from '../gamepadTypes'
import type { Buttons } from './standardMapping'

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
  const { armMode, currentValues, previousValues, gamepadBinds } = args
  const buttonsProfile = gamepadBinds.buttonFeatures

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
    if (pressed(translateGamepadButton(buttonsProfile, 'videoFeedPreviousArmMode'))) {
      actions.push(decreaseCamera(1))
    }

    if (pressed(translateGamepadButton(buttonsProfile, 'videoFeedNextArmMode'))) {
      actions.push(increaseCamera(1))
    }
  }

  if (armMode === ArmMode.IK) {
    // Inverse Kinematics
    const messageIK: ArmIkCommand = {
      // eslint-disable-next-line camelcase
      linear_x: translateGamepadAxis(currentValues, gamepadBinds, 'armLinearX'),
      // eslint-disable-next-line camelcase
      linear_y: translateGamepadAxis(currentValues, gamepadBinds, 'armLinearY'),
      // eslint-disable-next-line camelcase
      linear_z: translateGamepadAxis(currentValues, gamepadBinds, 'armLinearY'),
      // eslint-disable-next-line camelcase
      angular_x: translateGamepadAxis(currentValues, gamepadBinds, 'armAngularX'),
      // eslint-disable-next-line camelcase
      angular_y: translateGamepadAxis(currentValues, gamepadBinds, 'armAngularY'),
      // eslint-disable-next-line camelcase
      angular_z: translateGamepadAxis(currentValues, gamepadBinds, 'armAngularZ'),
    }
    actions.push(sendMessage({ topic: '/station/arm/ik/command', data: messageIK }))
  }

  if (armMode === ArmMode.FK) {
    // Forward Kinematics
    const messageFK = {
      gripper:
        currentValues.buttons[translateGamepadButton(buttonsProfile, 'openGripper')] -
        currentValues.buttons[translateGamepadButton(buttonsProfile, 'closeGripper')],
      // eslint-disable-next-line camelcase
      joint_1: translateGamepadAxis(currentValues, gamepadBinds, 'armJoint1'), // left horizontal, negative
      // eslint-disable-next-line camelcase
      joint_2: translateGamepadAxis(currentValues, gamepadBinds, 'armJoint2'), // left vertical, negative
      // eslint-disable-next-line camelcase
      joint_3: translateGamepadAxis(currentValues, gamepadBinds, 'armJoint3'), // right horizontal, negative
      // eslint-disable-next-line camelcase
      joint_4: translateGamepadAxis(currentValues, gamepadBinds, 'armJoint4'), // right vertical, positive
      // eslint-disable-next-line camelcase
      joint_5: translateGamepadButton(buttonsProfile, 'dpadUp') - translateGamepadButton(buttonsProfile, 'dpadDown'),
      // eslint-disable-next-line camelcase
      joint_6: translateGamepadAxis(currentValues, gamepadBinds, 'armJoint6'), // triggers
    }

    actions.push(sendMessage({ topic: '/station/arm/fk/command', data: messageFK }))
  }

  return actions
}
