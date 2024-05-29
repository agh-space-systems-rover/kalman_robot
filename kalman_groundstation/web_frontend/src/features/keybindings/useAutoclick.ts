import { useEffect, useRef, useState } from 'react'

import { setAutoclick, setScrewdriver } from '../../api/requests'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppSelector } from '../../store/storeHooks'

const autoclickKey = 'autoclickKey'
const autoclickExtendKey = 'autoclickExtendKey'
const screwdriverRight = 'screwdriverRight'
const screwdriverLeft = 'screwdriverLeft'
const screwdriverSpeedupKey = 'screwdriverSpeedupKey'

export const useAutoclick: () => void = () => {
  const [autoclickOffMessages, setAutoclickOffMessages] = useState<number>(0)
  const [screwdriverOffMessages, setScrewdriverOffMessages] = useState<number>(0)
  const intervalRef = useRef<NodeJS.Timer | null>(null)
  const screwdriverIntervalRef = useRef<NodeJS.Timer | null>(null)
  const pressedKeys = useAppSelector(selectKeys)

  useEffect(() => {
    if (pressedKeys.includes(autoclickKey)) {
      setAutoclickOffMessages(5)
      intervalRef.current = setInterval(() => {
        setAutoclick(pressedKeys.includes(autoclickExtendKey) ? 180 : 90)
      }, 250)
    } else {
      if (autoclickOffMessages > 0) {
        intervalRef.current = setInterval(() => {
          setAutoclick(0)
          setAutoclickOffMessages((prev) => prev - 1)
        }, 250)
      }
    }

    if (pressedKeys.includes(screwdriverRight) || pressedKeys.includes(screwdriverLeft)) {
      setScrewdriverOffMessages(5)
      screwdriverIntervalRef.current = setInterval(() => {
        setScrewdriver(pressedKeys.includes(screwdriverSpeedupKey) ? 45 : 20, pressedKeys.includes(screwdriverRight))
      }, 250)
    } else {
      if (screwdriverOffMessages > 0) {
        screwdriverIntervalRef.current = setInterval(() => {
          setScrewdriver(0, pressedKeys.includes(screwdriverRight))
          setScrewdriverOffMessages((prev) => prev - 1)
        }, 250)
      }
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current)
      }

      if (screwdriverIntervalRef.current) {
        clearInterval(screwdriverIntervalRef.current)
      }
    }
  }, [pressedKeys, autoclickOffMessages, screwdriverOffMessages])
}
