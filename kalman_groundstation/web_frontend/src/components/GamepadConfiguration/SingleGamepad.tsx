import type React from 'react'

import type { GamepadConfig } from '../../features/gamepads/gamepadTypes'
import { Target } from '../../features/gamepads/gamepadTypes'
import { changeTarget } from '../../store/Gamepads/gamepadSlice'
import { changeDrivingMode } from '../../store/Motors/motorsSlice'
import { DrivingMode } from '../../store/Motors/motorTypes'
import { useAppDispatch } from '../../store/storeHooks'

interface Props {
  gamepad: GamepadConfig
}

const SingleGamepad: React.FC<Props> = (props) => {
  const dispatch = useAppDispatch()
  const gamepad = props.gamepad

  const setTarget = (target: Target) => () => {
    dispatch(changeTarget({ index: gamepad.index, target: target }))
  }

  return (
    <div>
      <h4>Index: {props.gamepad.index}</h4>
      <p>Name: {props.gamepad.name}</p>
      <p>Target: {props.gamepad.target}</p>
      <div>
        Output:
        <button onClick={setTarget(Target.Wheels)}>wheels</button>
        <button onClick={setTarget(Target.Arm)}>arm</button>
        <button onClick={setTarget(Target.Drill)}>drill</button>
        <button onClick={setTarget(Target.None)}>disabled</button>
      </div>
      <div>
        <button
          hidden={props.gamepad.target !== Target.Wheels}
          onClick={(): void => dispatch(changeDrivingMode(DrivingMode.Digging))}
        >
          Enable digging
        </button>
      </div>
    </div>
  )
}

export { SingleGamepad }
