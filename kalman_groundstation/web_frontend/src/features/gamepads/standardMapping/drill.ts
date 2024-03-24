import { sendMessage } from '../../../components/Websocket/websocketSlice'
import type { GamepadChange } from '../gamepadTypes'

export interface DrillCommand {
  drill: number
  height: number
}

export const drillMapping: GamepadChange = (args) => {
  const { currentValues } = args

  const message: DrillCommand = {
    // eslint-disable-next-line camelcase
    drill: (-1 - currentValues.axes[2]) / 2 - (-1 - currentValues.axes[5]) / 2,
    // eslint-disable-next-line camelcase
    height: -currentValues.axes[1],
  }

  return [sendMessage({ topic: '/station/science/drill', data: message })]
}
