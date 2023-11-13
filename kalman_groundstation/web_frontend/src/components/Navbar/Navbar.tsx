import './Navbar.css'

import type React from 'react'

import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { useAppSelector } from '../../store/storeHooks'
import { selectWebsocket } from '../Websocket/websocketSlice'
import { HamburgerButton } from './HamburgerButton'

interface NavbarProps {
  autonomyView: boolean
  toggleAutonomyView: () => void
  sidebarView: boolean
  toggleSidebarView: () => void
}

const Navbar: React.FC<NavbarProps> = ({ autonomyView, toggleAutonomyView, sidebarView, toggleSidebarView }) => {
  const ws = useAppSelector(selectWebsocket)

  const websocketIcon = ws.connectionStatus == ConnectionStatus.Connected ? <>✓</> : <>☠</>

  return (
    <div className='navbar'>
      <HamburgerButton onClick={toggleSidebarView} checked={sidebarView} />
      <span>
        <span className='ws-status'>WS Status:</span>
        <span className='ws-icon'>{websocketIcon}</span>
      </span>
      <div>Kalman Ground Station</div>
      <button className='switch-button' onClick={toggleAutonomyView}>
        SWITCH TO {autonomyView ? 'MANUAL' : 'AUTONOMY'}
      </button>
    </div>
  )
}

export { Navbar }
