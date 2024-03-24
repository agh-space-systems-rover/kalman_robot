import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { UspParams } from '../../components/AutonomyFront/Usp'
import type { AutonomyGps, AutonomyImu, AutonomyPose, AutonomyState, AutonomyTag } from './autonomyTypes'

const initialState: AutonomyState = {
  imu: {
    alfa: 0,
    beta: 0,
    gamma: 0,
  },
  imuRaw: {
    alfa: 0,
    beta: 0,
    gamma: 0,
  },
  ueuos: 'unknown',
  gps: {
    position: {
      latitude: null,
      longitude: null,
      altitude: null,
    },
  },
  odom: {
    position: {
      x: null,
      y: null,
      z: null,
    },
  },
  map: {
    position: {
      x: null,
      y: null,
      z: null,
    },
  },
  supervisorState: -1,
  tags: [],
  usp: null,
}

const autonomySlice = createSlice({
  name: 'autonomy',
  initialState,
  reducers: {
    updateImu: (state, action: PayloadAction<AutonomyImu>) => {
      state.imu = action.payload
      state.imuRaw = action.payload
    },
    updateUeuos: (state, action: PayloadAction<string>) => {
      state.ueuos = action.payload
    },
    updateGps: (state, action: PayloadAction<AutonomyGps>) => {
      state.gps = action.payload
    },
    updateOdom: (state, action: PayloadAction<AutonomyPose>) => {
      state.odom = action.payload
    },
    updateMap: (state, action: PayloadAction<AutonomyPose>) => {
      state.map = action.payload
    },
    updateSupervisorState: (state, action: PayloadAction<number>) => {
      state.supervisorState = action.payload
    },
    updateTags: (state, action: PayloadAction<AutonomyTag[]>) => {
      state.tags = action.payload
    },
    updateUsp: (state, action: PayloadAction<UspParams | null>) => {
      state.usp = action.payload
    },
    updateWaypoints: (state, action: PayloadAction<number[] | null>) => {
      state.waypoints = action.payload
    },
    updateMoveBaseStatus: (state, action: PayloadAction<number | null>) => {
      if (action.payload === null) {
        return
      }
      state.moveBaseStatus = action.payload
    },
    updateBleSignal: (state, action: PayloadAction<number | null>) => {
      state.bleSignal = action.payload
    },
    updateAutonomy: (state, action: PayloadAction<AutonomyState>) => {
      const s = action.payload
      state.imu = s.imu
      state.imuRaw = s.imuRaw
      state.gps = s.gps
      state.odom = s.odom
      state.map = s.map
      state.supervisorState = s.supervisorState
      state.tags = s.tags
      state.ueuos = s.ueuos
      if (s.usp) state.usp = s.usp
      if (s.waypoints) state.waypoints = s.waypoints
      if (s.moveBaseStatus) state.moveBaseStatus = s.moveBaseStatus
    },
  },
})

export const {
  updateImu,
  updateUeuos,
  updateGps,
  updateOdom,
  updateMap,
  updateSupervisorState,
  updateTags,
  updateUsp,
  updateWaypoints,
  updateMoveBaseStatus,
  updateAutonomy,
  updateBleSignal,
} = autonomySlice.actions

export const autonomyReducer = autonomySlice.reducer
