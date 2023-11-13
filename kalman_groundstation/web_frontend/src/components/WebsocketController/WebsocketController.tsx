import './WebsocketController.css'

import { useAppDispatch, useAppSelector } from '../../store/storeHooks'
import { selectWebsocket, selectWsShouldReconnect, setReconnecting } from '../Websocket/websocketSlice'

const WebsocketController: () => JSX.Element = () => {
  const ws = useAppSelector(selectWebsocket)
  const dispatch = useAppDispatch()
  const reconnect = useAppSelector(selectWsShouldReconnect)

  return (
    <div style={{ gridArea: 'ws' }}>
      <h3>WebSocket Status:</h3>
      <div>Status: {ws.connectionStatus}</div>
      Reconnect
      <label className='switch'>
        <input type='checkbox' checked={reconnect} onChange={(): void => dispatch(setReconnecting(!reconnect))} />
        <span className='slider round'></span>
      </label>
    </div>
  )
}

export { WebsocketController }
