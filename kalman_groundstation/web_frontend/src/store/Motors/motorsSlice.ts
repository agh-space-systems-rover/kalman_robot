import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import { scalingPresets } from '../../features/gamepads/standardMapping/wheels'
import type { MotorsState, MotorsTemperature, Wheels, WheelScaling } from './motorTypes'
import { DrivingMode } from './motorTypes'

export const initialWheelsState: MotorsState = {
  settings: {
    drivingMode: DrivingMode.Normal,
    speed: 1,
    scalingFactor: scalingPresets[0],
  },
  wheels: {
    // eslint-disable-next-line camelcase
    target_motors: {
      // eslint-disable-next-line camelcase
      front_right: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      back_right: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      back_left: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      front_left: { velocity: 0, angle: 0 },
    },
    motors: {
      // eslint-disable-next-line camelcase
      front_right: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      back_right: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      back_left: { velocity: 0, angle: 0 },
      // eslint-disable-next-line camelcase
      front_left: { velocity: 0, angle: 0 },
    },
  },
  temperature: null,
}

const motorsSlice = createSlice({
  name: 'motors',
  initialState: initialWheelsState,
  reducers: {
    updateWheels: (state, action: PayloadAction<Wheels>) => {
      state.wheels = action.payload
    },
    changeDrivingMode: (state, action: PayloadAction<DrivingMode>) => {
      state.settings.drivingMode = action.payload
    },
    changeWheelsScalingFactor: (state, action: PayloadAction<WheelScaling>) => {
      state.settings.scalingFactor = action.payload
    },
    updateTemperature: (state, action: PayloadAction<MotorsTemperature>) => {
      state.temperature = action.payload
    },
  },
})

export const { updateWheels, changeDrivingMode, changeWheelsScalingFactor, updateTemperature } = motorsSlice.actions

export const motorsReducer = motorsSlice.reducer
