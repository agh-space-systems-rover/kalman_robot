import type React from 'react'

import { useAbortingTrajectories } from './useAbortingTrajectories'
import { useArmSpeedControl } from './useArmSpeedContorl'
import { useAutoclick } from './useAutoclick'
import { useAxisLock } from './useAxisLock'
import { useKeyboardFeedControl } from './useFeedControl'
import { useGripper } from './useGripper'
import { useKeybindings } from './useKeybindings'

interface Props {
  children: React.ReactNode
}

const KeybindingsProvider: React.FC<Props> = ({ children }) => {
  useKeybindings()
  useAxisLock()
  useArmSpeedControl()
  useGripper()
  useAutoclick()
  useAbortingTrajectories()
  useKeyboardFeedControl()
  return <>{children}</>
}

export { KeybindingsProvider }
