import { useEffect } from 'react'

import { abortNewTrajectories } from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'

export const useAbortingTrajectories: () => void = () => {
  const pressedKeys = useAppSelector((state) => state.keys.pressedKeys)

  useEffect(() => {
    if (pressedKeys.includes('Escape')) {
      abortNewTrajectories()
    }
  }, [pressedKeys])
}
