import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { RootState } from '../store'

export type KeyId = string

export interface KeysState {
  pressedKeys: KeyId[]
}

const initialState: KeysState = {
  pressedKeys: [],
}

const keysSlice = createSlice({
  name: 'keys',
  initialState,
  reducers: {
    addKey: (state, action: PayloadAction<KeyId>) => {
      if (!state.pressedKeys.includes(action.payload)) {
        state.pressedKeys.push(action.payload)
      }
    },
    removeKey: (state, action: PayloadAction<KeyId>) => {
      state.pressedKeys = state.pressedKeys.filter((value) => value !== action.payload)
    },
  },
})

export const { addKey, removeKey } = keysSlice.actions
export const keysReducer = keysSlice.reducer
export const selectKeys: (state: RootState) => string[] = (state: RootState) => state.keys.pressedKeys
