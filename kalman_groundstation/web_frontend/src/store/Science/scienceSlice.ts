import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { Module, ScienceState, SmartProbe } from './scienceTypes'
import { PanoramaStatus } from './scienceTypes'

const initialState: ScienceState = {
  atmosphere: {
    temperature: -1,
    humidity: -1,
    pressure: -1,
  },
  modules: [
    {
      temperature: -1,
    },
    {
      temperature: -1,
    },
    {
      temperature: -1,
    },
  ],
  cameraFeedback: 'unknown',
  cameraFeedbackTime: 82800000, // renders as 0:0:0.000
  ERCTemperature: -1,
  ERCWeight: -1,
  panoramaStatus: PanoramaStatus.INITIAL,
  smartProbe: {
    temperature: -1,
    hummidity: -1,
  },
}

const scienceSlice = createSlice({
  name: 'science',
  initialState,
  reducers: {
    updateModules: (state, action: PayloadAction<Module[]>) => {
      state.modules = action.payload
    },
    updateCameraFeedback: (state) => state,
    updatedCameraFeedback: (state, action: PayloadAction<number>) => {
      state.cameraFeedbackTime = action.payload
    },
    updateTemperature: (state, action: PayloadAction<number>) => {
      state.ERCTemperature = action.payload
    },
    updateWeight: (state, action: PayloadAction<number>) => {
      state.ERCWeight = action.payload
    },
    updatePanoramaStatus: (state, action: PayloadAction<PanoramaStatus>) => {
      state.panoramaStatus = action.payload
    },
    updateSmartProbe: (state, action: PayloadAction<SmartProbe>) => {
      state.smartProbe = action.payload
    },
  },
})

export const {
  updateModules,
  updateCameraFeedback,
  updatedCameraFeedback,
  updateTemperature,
  updateWeight,
  updatePanoramaStatus,
  updateSmartProbe,
} = scienceSlice.actions

export const scienceReducer = scienceSlice.reducer
