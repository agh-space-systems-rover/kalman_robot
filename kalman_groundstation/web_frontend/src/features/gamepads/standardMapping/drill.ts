import { sendMessage } from '../../../components/Websocket/websocketSlice'
import { translateGamepadAxis } from '../../../store/Keybinds/keybindsSlice'
import type { GamepadChange } from '../gamepadTypes'

export interface DrillCommand {
  drill: number
  height: number
  height2: number
}

export const drillMapping: GamepadChange = (args) => {
  // const { currentValues } = args
  const { currentValues, gamepadBinds } = args

  const message: DrillCommand = {
    // eslint-disable-next-line camelcase
    drill: translateGamepadAxis(currentValues, gamepadBinds, 'drillSpin'),
    // eslint-disable-next-line camelcase
    height: translateGamepadAxis(currentValues, gamepadBinds, 'drillHeight'),
    height2: translateGamepadAxis(currentValues, gamepadBinds, 'drillHeight2'),
  }

  return [sendMessage({ topic: '/station/science/drill', data: message })]
}
