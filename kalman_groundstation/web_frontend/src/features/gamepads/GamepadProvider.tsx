import type React from 'react'

import { useGamepads } from './useGamepads'

interface Props {
  children: React.ReactNode
}

const GamepadProvider: React.FC<Props> = ({ children }) => {
  useGamepads()
  return <>{children}</>
}

export { GamepadProvider }
