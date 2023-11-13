import './index.css'

import React from 'react'
import { createRoot } from 'react-dom/client'
import { Provider } from 'react-redux'

import { App } from './App'
import { WebsocketProvider } from './components/Websocket/WebsocketProvider'
import { GamepadProvider } from './features/gamepads/GamepadProvider'
import { KeybindingsProvider } from './features/keybindings/KeybindingsProvider'
import { store } from './store/store'

const container = document.getElementById('root')
const root = createRoot(container!)
root.render(
  <React.StrictMode>
    <Provider store={store}>
      <WebsocketProvider>
        <GamepadProvider>
          <KeybindingsProvider>
            <App />
          </KeybindingsProvider>
        </GamepadProvider>
      </WebsocketProvider>
    </Provider>
  </React.StrictMode>,
)
