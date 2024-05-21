import { useEffect } from 'react'

import { setDimensionLock } from '../../api/requests'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppSelector } from '../../store/storeHooks'
import { ConnectionStatus } from '../websocket/websocketTypes'
import type { AxisLockConfig } from './keybindingsTypes'

const allLocksFalse: AxisLockConfig = {
  lockX: false,
  lockY: false,
  lockZ: false,
  lockRotX: false,
  lockRotY: false,
  lockRotZ: false,
}

const allLocksTrue: AxisLockConfig = {
  lockX: true,
  lockY: true,
  lockZ: true,
  lockRotX: true,
  lockRotY: true,
  lockRotZ: true,
}

export interface AxisLock {
  key: string
  isActive: (conf: AxisLockConfig) => boolean
  set: (conf: AxisLockConfig, value: boolean) => AxisLockConfig
}

const locks: AxisLock[] = [
  {
    key: 'axisLockXKey',
    isActive: (conf) => conf.lockX,
    set: (conf, value) => ({ ...conf, lockX: value }),
  },
  {
    key: 'axisLockYKey',
    isActive: (conf) => conf.lockY,
    set: (conf, value) => ({ ...conf, lockY: value }),
  },
  {
    key: 'axisLockZKey',
    isActive: (conf) => conf.lockZ,
    set: (conf, value) => ({ ...conf, lockZ: value }),
  },
  {
    key: 'axisRotXKey',
    isActive: (conf) => conf.lockRotX,
    set: (conf, value) => ({ ...conf, lockRotX: value }),
  },
  {
    key: 'axisRotYKey',
    isActive: (conf) => conf.lockRotY,
    set: (conf, value) => ({ ...conf, lockRotY: value }),
  },
  {
    key: 'axisRotZKey',
    isActive: (conf) => conf.lockRotZ,
    set: (conf, value) => ({ ...conf, lockRotZ: value }),
  },
]

export const useAxisLock: () => void = () => {
  const pressedKeys = useAppSelector(selectKeys)
  const wsStatus = useAppSelector((state) => state.websocket.connectionStatus)
  const locksDisabled = useAppSelector((state) => state.settings.disableAxisLocks)
  //   const gamepads = useAppSelector(selectGamepad)

  // for (const gamepad of gamepads) {
  //     if (gamepad.target == Target.Arm) {
  //         locksDisabled = false
  //     }
  // }

  useEffect(() => {
    if (wsStatus === ConnectionStatus.Connected && !locksDisabled) {
      let sthChanged = false
      let newState = allLocksFalse
      for (const lock of locks) {
        if (pressedKeys.includes(lock.key) && !lock.isActive(newState)) {
          newState = lock.set(newState, true)
          sthChanged = true
        }

        if (!pressedKeys.includes(lock.key) && lock.isActive(newState)) {
          newState = lock.set(newState, false)
          sthChanged = true
        }
      }
      if (JSON.stringify(newState) === JSON.stringify(allLocksFalse)) {
        newState = allLocksTrue
        sthChanged = true
      }
      if (sthChanged) {
        setDimensionLock(newState)
      }
    }
  }, [pressedKeys])
}
