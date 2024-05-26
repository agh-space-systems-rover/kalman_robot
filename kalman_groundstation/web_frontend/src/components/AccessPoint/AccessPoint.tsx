import { useState } from 'react'
import styled from 'styled-components'

import { sendAutonomy, sendSilentMode, sendUeuosState } from '../../api/requests'
import { updateSsid } from '../../store/AccessPoint/accessPointSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'
import { sendMessage } from '../Websocket/websocketSlice'

const Wrapper = styled.div`
  padding: 10px;
`
const Button = styled.button`
  display: box;
  padding: 5px 10px 5px 10px;
  border-radius: 10px;
  margin: 5px 5px 5px 0px;
`

const Select = styled.select`
  padding: 5px;
  border-radius: 5px;
`

const enum AutonomyHint {
  Unknown,
  On,
  Off,
}

export const AccessPoint: () => JSX.Element = () => {
  const state = useAppSelector((state) => state.accessPoint)
  const dispatch = useAppDispatch()

  const options = ['Isaac_Asimov', 'Reactor']

  const [autonomyHint, setAutonomyHint] = useState<AutonomyHint>(AutonomyHint.Unknown)

  const submitAutonomyOn: () => void = () => {
    setAutonomyHint(AutonomyHint.On)
    sendAutonomy(true)
  }
  const submitAutonomyOff: () => void = () => {
    setAutonomyHint(AutonomyHint.Off)
    sendAutonomy(false)
  }

  const submitSilentModeOn: () => void = () => {
    sendSilentMode(true)
  }

  const submitSilentModeOff: () => void = () => {
    sendSilentMode(false)
  }

  const handleOptionChange: (event: React.ChangeEvent<HTMLSelectElement>) => void = (event) => {
    dispatch(updateSsid(event.target.value))
  }

  const connect: () => void = () => {
    dispatch(sendMessage({ topic: '/access_point/connect', data: { data: state.ssid } }))
  }

  const transfer: () => void = () => {
    dispatch(sendMessage({ topic: '/access_point/transfer', data: { data: state.ssid } }))
  }

  const lines = state.content.split('\n').map((s, i) => <div key={i}>{s}</div>)

  return (
    <Wrapper>
      <div>SSID</div>
      <Select value={state.ssid} onChange={handleOptionChange}>
        <option value=''>Select an option</option>
        {options.map((option, index) => (
          <option key={index} value={option}>
            {option}
          </option>
        ))}
      </Select>
      <div style={{ display: 'flex' }}>
        <Button onClick={connect} disabled={state.ssid === ''}>
          Connect
        </Button>
        <Button onClick={transfer} disabled={state.ssid === ''}>
          Transfer
        </Button>
      </div>
      <Wrapper>
        <div>Status: {state.status}</div>
        <div>Content:</div>
        <div>{lines}</div>
      </Wrapper>
      UEUOS:
      <br />
      <div>
        <button onClick={(): void => sendUeuosState(1)} style={{ margin: 3 }}>
          RED
        </button>
        <button onClick={(): void => sendUeuosState(2)} style={{ margin: 3 }}>
          BLUE
        </button>
        <button onClick={(): void => sendUeuosState(3)} style={{ margin: 3 }}>
          GREEN
        </button>
      </div>
      Silent mode:
      <br />
      <button onClick={(): void => submitSilentModeOn()} style={{ margin: 3 }}>
        Enable
      </button>
      <button onClick={(): void => submitSilentModeOff()} style={{ margin: 3 }}>
        Disable
      </button>
      <br />
      Autonomy state
      <br />
      <button
        onClick={(): void => submitAutonomyOn()}
        style={{
          border: `solid 2px ${autonomyHint === AutonomyHint.On ? '#ef5350' : '#ccc'}`,
          padding: '2px',
        }}
      >
        autonomy ON
      </button>
      <br />
      <button
        onClick={(): void => submitAutonomyOff()}
        style={{
          border: `solid 2px ${autonomyHint === AutonomyHint.Off ? '#ef5350' : '#ccc'}`,
          padding: '2px',
        }}
      >
        autonomy OFF
      </button>
    </Wrapper>
  )
}
