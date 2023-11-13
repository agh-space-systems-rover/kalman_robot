import { useEffect } from 'react'

import { setArmAngularVelocity, setArmLinearVelocity } from '../../api/requests'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppSelector } from '../../store/storeHooks'

export const useArmSpeedControl: () => void = () => {
  const pressedKeys = useAppSelector(selectKeys)

  useEffect(() => {
    if (pressedKeys.includes('KeyV')) {
      setArmLinearVelocity(0.3)
    }
    if (pressedKeys.includes('KeyB')) {
      setArmLinearVelocity(0.5)
    }
    if (pressedKeys.includes('KeyN')) {
      setArmLinearVelocity(0.7)
    }
    if (pressedKeys.includes('KeyF')) {
      setArmAngularVelocity(0.3)
    }
    if (pressedKeys.includes('KeyG')) {
      setArmAngularVelocity(0.5)
    }
    if (pressedKeys.includes('KeyH')) {
      setArmAngularVelocity(0.7)
    }
  }, [pressedKeys])
}
