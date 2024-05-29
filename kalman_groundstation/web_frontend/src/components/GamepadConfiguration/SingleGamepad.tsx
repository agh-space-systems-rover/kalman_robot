import type React from 'react'
import styled from 'styled-components'

import type { GamepadConfig } from '../../features/gamepads/gamepadTypes'
import { Target } from '../../features/gamepads/gamepadTypes'
import { changeTarget } from '../../store/Gamepads/gamepadSlice'
import { changeDrivingMode } from '../../store/Motors/motorsSlice'
import { DrivingMode } from '../../store/Motors/motorTypes'
import { useAppDispatch } from '../../store/storeHooks'

const Button = styled.button`
  margin: 5px;
  margin-top: 0;
  margin-bottom: 0;
  border: 1px solid #ccc;
`

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
        <Button onClick={setTarget(Target.Wheels)}>wheels</Button>
        <Button onClick={setTarget(Target.Arm)}>arm</Button>
        <Button onClick={setTarget(Target.Drill)}>drill</Button>
        <Button onClick={setTarget(Target.None)}>disabled</Button>
      </div>
      <div>
        <Button
          hidden={props.gamepad.target !== Target.Wheels}
          onClick={(): void => dispatch(changeDrivingMode(DrivingMode.Digging))}
        >
          Enable digging
        </Button>
      </div>
    </div>
  )
}

export { SingleGamepad }
