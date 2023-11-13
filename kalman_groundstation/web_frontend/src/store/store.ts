import { combineReducers, configureStore } from '@reduxjs/toolkit'

import { websocketMiddleware } from '../components/Websocket/websocketMiddleware'
import { websocketReducer } from '../components/Websocket/websocketSlice'
import { accessPointReducer } from './AccessPoint/accessPointSlice'
import { armReducer } from './Arm/armSlice'
import { autonomyReducer } from './Autonomy/autonomySlice'
import { feedReducer } from './Feeds/feedsSlice'
import { gamepadReducer } from './Gamepads/gamepadSlice'
import { keysReducer } from './Keys/keysSlice'
import { motorsReducer } from './Motors/motorsSlice'
import { scienceReducer } from './Science/scienceSlice'
import { settingsReducer } from './Settings/reducers'
import { trajectoriesReducer } from './Trajectories/trajectoriesSlice'

export const store = configureStore({
  reducer: combineReducers({
    gamepads: gamepadReducer,
    websocket: websocketReducer,
    science: scienceReducer,
    motors: motorsReducer,
    arm: armReducer,
    settings: settingsReducer,
    trajectories: trajectoriesReducer,
    keys: keysReducer,
    autonomy: autonomyReducer,
    feed: feedReducer,
    accessPoint: accessPointReducer,
  }),
  middleware: [websocketMiddleware],
})

export type AppDispatch = typeof store.dispatch
export type RootState = ReturnType<typeof store.getState>
