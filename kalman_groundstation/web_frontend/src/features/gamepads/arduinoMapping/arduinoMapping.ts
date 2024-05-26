import { changeTarget } from '../../../store/Gamepads/gamepadSlice'
import type { GamepadChange, MappingResult } from '../gamepadTypes'
import { Target } from '../gamepadTypes'
import { wheelsMapping } from './wheels'

export enum Buttons {
  RB = 2, // toggle inplace
  LB = 18, // toggle sideways
}

export const arduinoMapping: GamepadChange = (args) => {
  const { config } = args

  let response: MappingResult
  if (config.target === Target.Wheels) {
    response = wheelsMapping(args)
  } else {
    alert('jestem kontrolerem do kół debilu')
    response = [changeTarget({ index: config.index, target: Target.None })]
  }
  return response
}
