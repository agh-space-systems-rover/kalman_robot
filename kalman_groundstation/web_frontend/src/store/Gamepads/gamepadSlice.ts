import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { GamepadConfig } from '../../features/gamepads/gamepadTypes'
import { Side, Target } from '../../features/gamepads/gamepadTypes'
import type { RootState } from '../store'

const initialState: GamepadConfig[] = []

export const gamepadSlice = createSlice({
  name: 'gamepads',
  initialState,
  reducers: {
    addGamepad: (state, action: PayloadAction<GamepadConfig>) => {
      state.push(action.payload)
    },
    removeGamepad: (state, action: PayloadAction<number>) => {
      return state.filter((g) => g.index !== action.payload)
    },
    changeTarget: (state, action: PayloadAction<{ index: number; target: Target }>) => {
      const { index, target } = action.payload
      return state.map((g) => {
        if (g.index === index) {
          return { ...g, target: target }
        } else if (g.target === target) {
          return { ...g, target: Target.None }
        } else {
          return { ...g }
        }
      })
    },
    pressedTrigger: (state, action: PayloadAction<{ index: number; side: Side }>) => {
      const { index, side } = action.payload
      if (side == Side.Right) {
        return state.map((g) => (g.index === index ? { ...g, rightTriggerPressed: true } : { ...g }))
      } else {
        return state.map((g) => (g.index === index ? { ...g, leftTriggerPressed: true } : { ...g }))
      }
    },
  },
})

export const { addGamepad, removeGamepad, changeTarget, pressedTrigger } = gamepadSlice.actions
export const gamepadReducer = gamepadSlice.reducer

export const selectGamepad: (state: RootState) => GamepadConfig[] = (state: RootState) => state.gamepads
