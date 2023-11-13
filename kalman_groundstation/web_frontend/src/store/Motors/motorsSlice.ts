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
    targetMotors: {
      fr: { velocity: 0, angle: 0 },
      br: { velocity: 0, angle: 0 },
      bl: { velocity: 0, angle: 0 },
      fl: { velocity: 0, angle: 0 },
    },
    motors: {
      fr: { velocity: 0, angle: 0 },
      br: { velocity: 0, angle: 0 },
      bl: { velocity: 0, angle: 0 },
      fl: { velocity: 0, angle: 0 },
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
