import { useEffect } from 'react'
import styled from 'styled-components'

import { abortNewTrajectories, executeNewTrajectory, getNewTrajectories } from '../../api/requests'
import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { useAppSelector } from '../../store/storeHooks'

const Wrapper = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  &:nth-child(2n) {
    background: rgba(0, 0, 0, 0.05);
  }
`

const AbortButton = styled.div`
  font-size: 30px;
`
export const NewTrajectories: () => JSX.Element = () => {
  const list = useAppSelector((state) => [...state.trajectories.newTrajectories].sort((a, b) => a.localeCompare(b)))
  const wsState = useAppSelector((state) => state.websocket.connectionStatus)

  useEffect(() => {
    if (wsState == ConnectionStatus.Connected) {
      getNewTrajectories()
    }
  }, [wsState])

  const elements = list.map((item) => (
    <Wrapper key={item}>
      <div>
        <button type='button' onClick={(): void => executeNewTrajectory(item)}>
          {' '}
          Execute{' '}
        </button>{' '}
        {item}
      </div>
    </Wrapper>
  ))

  return (
    <div>
      <h3>Trajectories (new)</h3>
      {elements}
      {list.length > 0 && (
        <AbortButton>
          <button type='button' onClick={abortNewTrajectories}>
            <h3>Abort all</h3>
          </button>
        </AbortButton>
      )}
    </div>
  )
}
