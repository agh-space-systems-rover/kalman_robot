import { useEffect, useState } from 'react'

import { changeTarget, selectGamepad } from '../../store/Gamepads/gamepadSlice'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'
import { Target } from '../gamepads/gamepadTypes'

const rightKey = 'Period'
const leftKey = 'Comma'
const messageFromTemplate: (value: number) => string = (value) => {
  // eslint-disable-next-line max-len
  return `{"target":1,"data":{"mode":1,"gripper":${value},"data":{"joint_1":0,"joint_2":0,"joint_3":0,"joint_4":0,"joint_5":0,"joint_6":0}}}`
}

export const useGripper: () => void = () => {
  const dispatch = useAppDispatch()
  const pressedKeys = useAppSelector(selectKeys)
  const gamepads = useAppSelector(selectGamepad)
  const [armGamepadIndex, setArmGamepadIndex] = useState<number | null>(null)
  //   const interval = useAppSelector((state) => state.settings.refreshRate)
  const gripperDisabled = useAppSelector((state) => state.settings.disableGripper)

  useEffect(() => {
    if (gripperDisabled) return
    const leftKeyPressed = pressedKeys.includes(leftKey)
    const rightKeyPressed = pressedKeys.includes(rightKey)

    if ((leftKeyPressed && !rightKeyPressed) || (!leftKeyPressed && rightKeyPressed)) {
      let gamepadAlreadyStopped = true

      for (const gamepad of gamepads) {
        if (gamepad.target === Target.Arm) {
          gamepadAlreadyStopped = false
          dispatch(changeTarget({ index: gamepad.index, target: Target.None }))
          setArmGamepadIndex(gamepad.index)
        }
      }

      if (gamepadAlreadyStopped) {
        const direction = leftKeyPressed ? 'left' : 'right'
        console.log('Start rotating gripper ' + direction)

        const message = messageFromTemplate(leftKeyPressed ? -1 : 1)
        // const messageInterval = setInterval(() => dispatch(sendMessage(message)), interval)

        return () => {
          console.log('Stop rotating ' + message)
          // clearInterval(messageInterval)

          if (armGamepadIndex != null) {
            dispatch(changeTarget({ index: armGamepadIndex, target: Target.Arm }))
            setArmGamepadIndex(null)
          }
        }
      }
    }
    return
  }, [pressedKeys, gamepads, armGamepadIndex])
}
