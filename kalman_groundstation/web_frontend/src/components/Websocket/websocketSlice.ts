import type { PayloadAction } from '@reduxjs/toolkit'
import { createSlice } from '@reduxjs/toolkit'

import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import type { RootState } from '../../store/store'

const initialState = {
  connectionStatus: ConnectionStatus.Disconnected,
  shouldReconnect: true,
}

export interface WebsocketMessage {
  topic: string
  data: unknown
}

const websocketSlice = createSlice({
  name: 'websocket',
  initialState,
  reducers: {
    setReconnecting: (state, action: PayloadAction<boolean>) => {
      state.shouldReconnect = action.payload
    },
    startConnecting: (state) => {
      state.connectionStatus = ConnectionStatus.Connecting
    },
    connectionEstablished: (state) => {
      state.connectionStatus = ConnectionStatus.Connected
    },
    startDisconnecting: (state) => {
      state.connectionStatus = ConnectionStatus.Disconnecting
    },
    connectionTerminated: (state) => {
      state.connectionStatus = ConnectionStatus.Disconnected
    },
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    sendMessage: (state, action: PayloadAction<WebsocketMessage>) => {
      // stops action to avoid spamming store
      return
    },
  },
})

export const {
  startConnecting,
  connectionEstablished,
  sendMessage,
  startDisconnecting,
  connectionTerminated,
  setReconnecting,
} = websocketSlice.actions
export const websocketReducer = websocketSlice.reducer

export const selectWebsocket: (state: RootState) => {
  connectionStatus: ConnectionStatus
} = (state: RootState) => state.websocket

export const selectWsShouldReconnect: (state: RootState) => boolean = (state) => state.websocket.shouldReconnect
