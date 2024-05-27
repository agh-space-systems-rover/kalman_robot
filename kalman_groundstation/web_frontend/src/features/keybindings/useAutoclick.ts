import { useEffect, useRef, useState } from 'react'

import { setAutoclick } from '../../api/requests'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppSelector } from '../../store/storeHooks'

const autoclickKey = 'autoclickKey'

export const useAutoclick: () => void = () => {
  const [autoclickOffMessages, setAutoclickOffMessages] = useState<number>(0)
  const intervalRef = useRef<NodeJS.Timer | null>(null)
  const pressedKeys = useAppSelector(selectKeys)

  useEffect(() => {
    if (pressedKeys.includes(autoclickKey)) {
      setAutoclickOffMessages(5)
      intervalRef.current = setInterval(() => {
        setAutoclick(true)
      }, 250)
    } else {
      if (autoclickOffMessages > 0) {
        intervalRef.current = setInterval(() => {
          setAutoclick(false)
          setAutoclickOffMessages((prev) => prev - 1)
        }, 250)
      }
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current)
      }
    }
  }, [pressedKeys, autoclickOffMessages])
}
