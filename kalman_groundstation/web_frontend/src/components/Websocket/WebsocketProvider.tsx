import type React from 'react'
import { useEffect } from 'react'

import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'
import { selectWebsocket, selectWsShouldReconnect, startConnecting, startDisconnecting } from './websocketSlice'

interface Props {
  children: React.ReactNode
}

export const WebsocketProvider: React.FC<Props> = ({ children }) => {
  const reconnect = useAppSelector(selectWsShouldReconnect)
  const { connectionStatus } = useAppSelector(selectWebsocket)
  const dispatch = useAppDispatch()

  useEffect(() => {
    let timeout: NodeJS.Timeout | null = null

    // reconnect after being disconnected for 2 seconds
    if (
      reconnect &&
      (connectionStatus === ConnectionStatus.Disconnected || connectionStatus === ConnectionStatus.Disconnecting)
    ) {
      timeout = setTimeout(() => {
        dispatch(startConnecting())
      }, 2000)
    }

    // cancel connecting after 3 seconds
    if (reconnect && connectionStatus === ConnectionStatus.Connecting) {
      timeout = setTimeout(() => {
        dispatch(startDisconnecting())
      }, 3000)
    }

    // disconnect when toggle is off to be able to trigger reconnect manually
    if (
      !reconnect &&
      (connectionStatus === ConnectionStatus.Connected || connectionStatus === ConnectionStatus.Connecting)
    ) {
      dispatch(startDisconnecting())
    }

    return () => {
      if (timeout) {
        clearTimeout(timeout)
      }
    }
  }, [reconnect, connectionStatus])
  return <>{children}</>
}
