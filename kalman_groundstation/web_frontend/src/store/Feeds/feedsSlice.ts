import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import type { RootState } from '../store'

export interface Feed {
  camera: number
  channel: number
  power: number
}

export interface FeedState {
  feeds: Feed[]
}

export const initialFeedState: FeedState = {
  feeds: [
    {
      camera: 1,
      channel: 1,
      power: 1,
    },
    {
      camera: 2,
      channel: 1,
      power: 1,
    },
  ],
}

const feedSlice = createSlice({
  name: 'feeds',
  initialState: initialFeedState,
  reducers: {
    setFeeds: (state, action: PayloadAction<Feed[]>) => {
      state.feeds = action.payload
    },
    increaseCamera: (state, action: PayloadAction<number>) => {
      state.feeds[action.payload].camera += state.feeds[action.payload].camera < 8 ? 1 : 0
    },
    decreaseCamera: (state, action: PayloadAction<number>) => {
      state.feeds[action.payload].camera -= state.feeds[action.payload].camera > 0 ? 1 : 0
    },
  },
})

export const { setFeeds, increaseCamera, decreaseCamera } = feedSlice.actions

export const feedReducer = feedSlice.reducer
export const selectFeeds: (state: RootState) => Feed[] = (state: RootState) => state.feed.feeds
