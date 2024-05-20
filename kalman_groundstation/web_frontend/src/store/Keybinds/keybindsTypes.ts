export interface keyMapping {
  key: string
  target: string
}

export interface keyBinds {
  keybinds: keyMapping[]
}

export interface userProfile {
  username: string
  name: string
  binds: keyMapping[]
  gamepadBinds: gamepadSettings
}

export interface gamepadFeatureButton {
  button: number // maybe enum would be more appropriate here
  target: string
}

// for arduino's map
export interface valueRange {
  low: number
  high: number
}

export interface mappingParameters {
  inputRange: valueRange
  outputRange: valueRange
  clipping: boolean
}

export interface gamepadFeatureAxis {
  axis: number
  target: string
  valueMapping: mappingParameters
}

export interface gamepadVirtualAxis {
  target: string
  sourceAxes: string[]
  clipping: boolean
}

export interface gamepadSettings {
  buttonFeatures: gamepadFeatureButton[]
  axisFeatures: gamepadFeatureAxis[]
  virtualAxes: gamepadVirtualAxis[]
}

export const noMapping: mappingParameters = {
  // TODO ask about nonlinear mapping
  inputRange: {
    low: -1,
    high: 1,
  },
  outputRange: {
    low: -1,
    high: 1,
  },
  clipping: false,
}

export const invertedMapping: mappingParameters = {
  // TODO ask about nonlinear mapping
  inputRange: {
    low: -1,
    high: 1,
  },
  outputRange: {
    low: 1,
    high: -1,
  },
  clipping: false,
}

export const initialUserProfile: userProfile = {
  username: 'default',
  name: 'Default user',
  binds: [
    { target: 'armSlowVelocityLinearKey', key: 'KeyV' },
    { target: 'armMediumVelocityLinearKey', key: 'KeyB' },
    { target: 'armFastVelocityLinearKey', key: 'KeyN' },

    { target: 'armSlowVelocityAngularKey', key: 'KeyF' },
    { target: 'armMediumVelocityAngularKey', key: 'KeyG' },
    { target: 'armFastVelocityAngularKey', key: 'KeyH' },

    { target: 'feed1.1Key', key: 'Digit1' },
    { target: 'feed1.2Key', key: 'Digit2' },
    { target: 'feed1.3Key', key: 'Digit3' },
    { target: 'feed1.4Key', key: 'Digit4' },
    { target: 'feed1.5Key', key: 'Digit5' },
    { target: 'feed1.6Key', key: 'Digit6' },
    { target: 'feed1.7Key', key: 'Digit7' },
    { target: 'feed1.8Key', key: 'Digit8' },

    { target: 'feed2.1Key', key: 'KeyQ' },
    { target: 'feed2.2Key', key: 'KeyW' },
    { target: 'feed2.3Key', key: 'KeyE' },
    { target: 'feed2.4Key', key: 'KeyR' },
    { target: 'feed2.5Key', key: 'KeyT' },
    { target: 'feed2.6Key', key: 'KeyY' },
    { target: 'feed2.7Key', key: 'KeyU' },
    { target: 'feed2.8Key', key: 'KeyI' },

    { target: 'axisLockXKey', key: 'KeyA' },
    { target: 'axisLockYKey', key: 'KeyS' },
    { target: 'axisLockZKey', key: 'KeyD' },
    { target: 'axisRotXKey', key: 'KeyZ' },
    { target: 'axisRotYKey', key: 'KeyX' },
    { target: 'axisRotZKey', key: 'KeyC' },

    { target: 'openGripperKey', key: 'Period' },
    { target: 'closeGripperKey', key: 'Comma' },
  ],
  gamepadBinds: {
    buttonFeatures: [
      { button: 0, target: 'scaleSpeedDown' }, // todo dodać jakiś enum
      { button: 2, target: 'scaleSpeedUp' },
      { button: 5, target: 'drivingModeInPlace' },
      { button: 4, target: 'drivingModeSideways' },

      { button: 1, target: 'videoFeedNextWheelsMode' },
      { button: 3, target: 'videoFeedPreviousWheelsMode' },
      { button: 1, target: 'videoFeedNextArmMode' },
      { button: 3, target: 'videoFeedPreviousArmMode' },

      { button: 1, target: 'closeGripper' },
      { button: 0, target: 'openGripper' },
    ],
    axisFeatures: [
      // driving mode normal
      { axis: 2, target: 'normalTurnAxis', valueMapping: invertedMapping },
      {
        axis: 0,
        target: 'offsetAxis',
        valueMapping: {
          inputRange: {
            low: -1,
            high: 1,
          },
          outputRange: {
            low: 2,
            high: -2,
          },
          clipping: false,
        },
      },
      // driving mode in place
      {
        axis: 2,
        target: 'inPlaceTurnAxis',
        valueMapping: {
          inputRange: {
            low: -1,
            high: 1,
          },
          outputRange: {
            low: 2,
            high: -2,
          },
          clipping: false,
        },
      },
      { axis: 0, target: 'inPlaceSomethingAxis', valueMapping: noMapping },

      // driving mode sideways
      { axis: 3, target: 'sidewaysTurnAxis', valueMapping: noMapping },
      { axis: 1, target: 'sidewaysShitAxis', valueMapping: noMapping },

      // arm linear ik
      { axis: 2, target: 'armLinearX', valueMapping: noMapping },
      { axis: 3, target: 'armLinearY', valueMapping: noMapping },
      // armLinearZ is virtual

      // arm angular ik
      { axis: 0, target: 'armAngularX', valueMapping: noMapping },
      { axis: 1, target: 'armAngularY', valueMapping: noMapping },
      { axis: 2, target: 'armAngularZ', valueMapping: noMapping },

      // arm fk
      { axis: 0, target: 'armJoint1', valueMapping: invertedMapping },
      { axis: 1, target: 'armJoint2', valueMapping: invertedMapping },
      { axis: 2, target: 'armJoint3', valueMapping: invertedMapping },
      { axis: 3, target: 'armJoint4', valueMapping: noMapping },
      { axis: 7, target: 'armJoint5', valueMapping: invertedMapping },
      // armJoint6 is virtual

      // shit
      { axis: 3, target: 'messageY', valueMapping: noMapping },
      { axis: 2, target: 'messageZ', valueMapping: noMapping }, // SPRAWDZIĆ, TO JEST NA CZUJA!!!

      // virtual axes components
      {
        axis: 4,
        target: '_negativeTriggerAxis',

        valueMapping: {
          inputRange: {
            low: -1,
            high: 1,
          },
          outputRange: {
            low: 0,
            high: -1,
          },
          clipping: false,
        },
      },
      {
        axis: 5,
        target: '_positiveTriggerAxis',

        valueMapping: {
          inputRange: {
            low: -1,
            high: 1,
          },
          outputRange: {
            low: 0,
            high: 1,
          },
          clipping: false,
        },
      },
    ],
    virtualAxes: [
      {
        target: 'throttleBrake',
        sourceAxes: ['_negativeTriggerAxis', '_positiveTriggerAxis'],
        clipping: false,
      },
      {
        target: 'armLinearZ',
        sourceAxes: ['_negativeTriggerAxis', '_positiveTriggerAxis'],
        clipping: false,
      },
      {
        target: 'armJoint6',
        sourceAxes: ['_negativeTriggerAxis', '_positiveTriggerAxis'],
        clipping: false,
      },
    ],
  },
}

// axes
// 0: left stick, horizontal
// 1: left stick, vertical
// 2: right stick, horizontal
// 3: right stick, vertical
// 5: left trigger
// 6: right trigger
