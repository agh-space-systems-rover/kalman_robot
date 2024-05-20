import { useEffect } from 'react'

import { setArmAngularVelocity, setArmLinearVelocity } from '../../api/requests'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppSelector } from '../../store/storeHooks'

export const useArmSpeedControl: () => void = () => {
  const pressedKeys = useAppSelector(selectKeys)

  useEffect(() => {
    if (pressedKeys.includes('armSlowVelocityLinearKey')) {
      // KeyV
      setArmLinearVelocity(0.3)
    }
    if (pressedKeys.includes('armMediumVelocityLinearKey')) {
      setArmLinearVelocity(0.5)
    }
    if (pressedKeys.includes('armFastVelocityLinearKey')) {
      setArmLinearVelocity(0.7)
    }
    if (pressedKeys.includes('armSlowVelocityAngularKey')) {
      setArmAngularVelocity(0.3)
    }
    if (pressedKeys.includes('armMediumVelocityAngularKey')) {
      setArmAngularVelocity(0.5)
    }
    if (pressedKeys.includes('armFastVelocityAngularKey')) {
      setArmAngularVelocity(0.7)
    }
  }, [pressedKeys])
}
