import type { Action, Middleware, PayloadAction } from '@reduxjs/toolkit'

import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { updateContent, updateStatus } from '../../store/AccessPoint/accessPointSlice'
import { updateJoints } from '../../store/Arm/armSlice'
import { convertJointPositions } from '../../store/Arm/armTypes'
import { updateAutonomy, updateBleSignal } from '../../store/Autonomy/autonomySlice'
import { updateTemperature, updateWheels } from '../../store/Motors/motorsSlice'
import { updateModules, updateSmartProbe, updateUniversal, updateWeight } from '../../store/Science/scienceSlice'
import { mapScience, mapScienceUniversal, mapSmartProbe } from '../../store/Science/scienceTypes'
import {
  connectionEstablished,
  connectionTerminated,
  sendMessage,
  startConnecting,
  startDisconnecting,
} from './websocketSlice'

interface ReceivedMessage {
  topic: string
  data: any
}

const wheelHandler: (data: any) => Action = (data: any) => {
  return updateWheels(data)
}

const armHandler: (data: any) => Action = (data: any) => {
  return updateJoints(convertJointPositions(data))
}

const scienceHandler: (data: any) => Action = (data: any) => {
  return updateModules(mapScience(data))
}

const autonomyHandler: (data: any) => Action = (data: any) => {
  const obj = JSON.parse(data.data)
  return updateAutonomy(obj)
}

const temperatureHandler: (data: any) => Action = (data: any) => {
  return updateTemperature(data)
}

const weightHandler: (data: any) => Action = (data: any) => {
  return updateWeight(data.data)
}

const smartProbeHandler: (data: any) => Action = (data: any) => {
  console.log(data)
  return updateSmartProbe(mapSmartProbe(data))
}

const accessPointContentHandler: (data: any) => Action = (data: any) => {
  console.log(data.data)
  return updateContent(data.data)
}

const accessPointStatusHandler: (data: any) => Action = (data: any) => {
  console.log(data.data)
  return updateStatus(data.data)
}

const bleSignalHandler: (data: any) => Action = (data: any) => {
  return updateBleSignal(data.data)
}

const scienceRespHandler: (data: any) => Action = (data: any) => {
  return updateUniversal(mapScienceUniversal(data))
}

const handlers: any = {
  '/station/wheels/state': wheelHandler,
  '/station/arm/state': armHandler,
  '/station/autonomy/state': autonomyHandler,
  '/station/science/state': scienceHandler,
  '/station/science/weight': weightHandler,
  '/station/science/smart_probe': smartProbeHandler,
  '/station/wheels/temperatures': temperatureHandler,
  '/access_point/webpage/joined': accessPointContentHandler,
  '/access_point/status': accessPointStatusHandler,
  '/kalman_rover/ble_beacon_signal': bleSignalHandler,
  '/station/science/resp': scienceRespHandler,
}

const URL = 'ws://localhost:8001/ws'

export const websocketMiddleware: Middleware = (store) => {
  let socket: WebSocket
  return (next) => (action: PayloadAction<string>) => {
    const isConnectionEstablished = socket && store.getState().websocket.connectionStatus === ConnectionStatus.Connected
    if (startConnecting.match(action)) {
      socket = new WebSocket(URL)

      socket.onopen = (): void => {
        store.dispatch(connectionEstablished())
        console.log('Web Socket: Connected')
      }

      socket.onclose = (): void => {
        store.dispatch(connectionTerminated())
        console.log('Web Socket: Disconnected')
      }

      socket.onmessage = (e: MessageEvent): void => {
        const obj: ReceivedMessage = JSON.parse(e.data)

        if (obj.topic in handlers) {
          store.dispatch(handlers[obj.topic](obj.data))
        } else {
          console.warn('Received message to unregistered topic from websocket')
        }
      }
    }

    if (startDisconnecting.match(action) && isConnectionEstablished) {
      socket.close()
    }

    if (sendMessage.match(action) && isConnectionEstablished) {
      console.log(action.payload)
      socket.send(JSON.stringify(action.payload))
    }
    next(action)
  }
}
