import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { RootState } from '../store'
import type { NewTrajectory, Trajectories, TrajectoriesState } from './trajectoriesTypes'

const initialState: TrajectoriesState = {
  fetched: false,
  trajectories: {
    cartesianSpaceGoals: [],
    jointSpaceGoals: [],
  },
  newTrajectories: [],
}

const trajectoriesSlice = createSlice({
  name: 'trajectories',
  initialState,
  reducers: {
    setTrajectories: (state, action: PayloadAction<Trajectories>) => {
      state.trajectories = action.payload
      state.fetched = true
    },
    setNewTrajectories: (state, action: PayloadAction<NewTrajectory[]>) => {
      state.newTrajectories = action.payload
    },
  },
})

export const selectTrajectories: (state: RootState) => Trajectories = (state: RootState) =>
  state.trajectories.trajectories
export const selectTrajectoriesRequestState: (state: RootState) => boolean = (state: RootState) =>
  state.trajectories.fetched

export const { setTrajectories, setNewTrajectories } = trajectoriesSlice.actions

export const trajectoriesReducer = trajectoriesSlice.reducer
