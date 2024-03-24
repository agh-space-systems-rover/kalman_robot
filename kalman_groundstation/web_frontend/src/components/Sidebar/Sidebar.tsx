import './Sidebar.css'

import type React from 'react'

import { FeedControl } from '../FeedControl/FeedControl'
import { GamepadConfigurator } from '../GamepadConfiguration/GamepadConfigurator'
import { Settings } from '../Settings/Settings'
import { WebsocketController } from '../WebsocketController/WebsocketController'

const Sidebar: React.FC = () => {
  return (
    <div className='sidebar'>
      <GamepadConfigurator />
      <hr style={{ backgroundColor: 'black', height: 1 }} />
      <WebsocketController />
      <hr style={{ backgroundColor: 'black', height: 1 }} />
      <Settings />
      <hr style={{ backgroundColor: 'black', height: 1 }} />
      <FeedControl />
    </div>
  )
}

export { Sidebar }
