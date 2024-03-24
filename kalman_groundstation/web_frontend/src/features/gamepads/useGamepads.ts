import { useEffect, useRef } from 'react'

import { addGamepad, changeTarget, removeGamepad, selectGamepad } from '../../store/Gamepads/gamepadSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'
import { arduinoMapping } from './arduinoMapping/arduinoMapping'
import type { GamepadConfig, GamepadValues, MappingArguments, MappingResult } from './gamepadTypes'
import { Target } from './gamepadTypes'
import { standardMapping } from './standardMapping/standardMapping'
import { applyDeadzone } from './utils'

// This hook creates a loop that periodically reads and process values from attached gamepad.
// Within this loop several parameters from the redux store are used (gamepad configuration, arm settings, etc)
// Every time parameter changes a new loop is created. Each gamepad has it's own loop

const useGamepads: () => void = () => {
  const buttonsHistory = useRef(new Map<number, GamepadValues>())
  const gamepads = useAppSelector(selectGamepad)
  const motors = useAppSelector((s) => s.motors.settings)
  const arm = useAppSelector((s) => s.arm.armMode)
  const refreshRate = useAppSelector((s) => s.settings.refreshRate)
  const dispatch = useAppDispatch()

  useEffect(() => {
    const isGamePadEvent = (e: Event): e is GamepadEvent => {
      return 'gamepad' in e
    }

    const onGamepadConnected: (e: Event) => void = (e) => {
      if (isGamePadEvent(e)) {
        console.log('Gamepad connected')

        const newGamepad: GamepadConfig = {
          index: e.gamepad.index,
          name: e.gamepad.id,
          target: Target.None,
          leftTriggerPressed: false,
          rightTriggerPressed: false,
        }

        dispatch(addGamepad(newGamepad))
      }
    }

    const onGamepadDisconnected: (e: Event) => void = (e) => {
      if (isGamePadEvent(e)) {
        console.log('Gamepad disconnected')
        dispatch(removeGamepad(e.gamepad.index))
      }
    }

    const gamepadLoop = (gamepad: GamepadConfig) => () => {
      // get current gamepad state from browser api
      const gp = navigator.getGamepads()[gamepad.index]

      if (!gp) {
        return
      }

      if (gamepad.target != Target.None) {
        const currentValues: GamepadValues = {
          axes: applyDeadzone(gp.axes),
          buttons: gp.buttons.map((g) => g.value),
        }

        // for detecting button presses values from the previous iteration are necessary
        const previousValues = buttonsHistory.current.get(gamepad.index)!

        // parameters and gamepad values that can be used in mapping function
        const mappingArguments: MappingArguments = {
          config: gamepad,
          wheels: motors,
          armMode: arm,
          currentValues,
          previousValues,
        }

        // mapping produces:
        // - string message to be send to backend
        // - list of redux actions to be dispatched

        // check if pad is arduino
        let actions: MappingResult
        if (gp.id === '2341-8036-Arduino Leonardo' || gp.id === '2341-8036-Arduino LLC Arduino Leonardo') {
          actions = arduinoMapping(mappingArguments)
        } else {
          actions = standardMapping(mappingArguments)
        }

        actions.forEach((action) => dispatch(action))

        // send string message to backend through websocket middleware
        // dispatch(sendMessage(message))
        buttonsHistory.current.set(gamepad.index, currentValues)
      } else {
        if (gp.buttons[6].pressed == true) {
          dispatch(changeTarget({ index: gamepad.index, target: Target.Wheels }))
        } else if (gp.buttons[7].pressed == true) {
          dispatch(changeTarget({ index: gamepad.index, target: Target.Arm }))
        }
      }
    }

    // set up a loop for every gamepad that is not disabled (arm or wheels selected as target)
    const intervals = gamepads.map((g) => setInterval(gamepadLoop(g), 1000 / refreshRate))

    window.addEventListener('gamepadconnected', onGamepadConnected)
    window.addEventListener('gamepaddisconnected', onGamepadDisconnected)
    return () => {
      // stop running loops
      // most often it means that some parameter in redux store has changed and a new loop is required
      intervals.forEach((i) => clearInterval(i))
      window.removeEventListener('gamepadconnected', onGamepadConnected)
      window.removeEventListener('gamepaddisconnected', onGamepadDisconnected)
    }
  })
}

export { useGamepads }
