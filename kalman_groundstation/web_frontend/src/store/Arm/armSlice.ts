import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { ArmMode, JointPositions, KeyboardTypingMode } from './armTypes'
import { initialArmState } from './armTypes'

const armSlice = createSlice({
  name: 'arm',
  initialState: initialArmState,
  reducers: {
    updateJoints: (state, action: PayloadAction<JointPositions>) => {
      state.jointPositions = action.payload
    },
    changeArmMode: (state, action: PayloadAction<ArmMode>) => {
      state.armMode = action.payload
    },
    changeKeyboardTypingMode: (state, action: PayloadAction<KeyboardTypingMode>) => {
      state.keyboardTypingMode = action.payload
    },
  },
})

export const { updateJoints, changeArmMode, changeKeyboardTypingMode } = armSlice.actions

export const armReducer = armSlice.reducer
