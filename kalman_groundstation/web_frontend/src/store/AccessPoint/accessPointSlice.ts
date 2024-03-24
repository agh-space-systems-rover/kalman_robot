import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import { initialAccessPointState } from './accessPointTypes'

const AccessPointSlice = createSlice({
  name: 'arm',
  initialState: initialAccessPointState,
  reducers: {
    updateSsid: (state, action: PayloadAction<string>) => {
      state.ssid = action.payload
    },
    updateStatus: (state, action: PayloadAction<string>) => {
      state.status = action.payload
    },
    updateContent: (state, action: PayloadAction<string>) => {
      state.content = action.payload
    },
  },
})

export const { updateSsid, updateStatus, updateContent } = AccessPointSlice.actions

export const accessPointReducer = AccessPointSlice.reducer
