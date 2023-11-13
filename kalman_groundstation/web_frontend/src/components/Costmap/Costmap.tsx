import { useEffect, useState } from 'react'
import styled from 'styled-components'

import { requestCostmap } from '../../api/requests'
import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { useAppSelector } from '../../store/storeHooks'
import { infoDiv, Row } from './CostmapUtils'

const Wrapper = styled.div`
  grid-area: costmap;
`

export const cellColors = {
  OBSTACLE: '#f2023a',
  EMPTY: '#74a11b',
  UNKNOWN: '#808285',
}

export const Costmap: () => JSX.Element = () => {
  const unknownCostmapValues = Array(20)
    .fill([])
    .map(() => Array(20).fill(0))

  const [data, setData] = useState<number[][]>(unknownCostmapValues)
  const [continuosMode, setContinuosMode] = useState(false)

  const wsStatus = useAppSelector((state) => state.websocket.connectionStatus)
  const backURL = useAppSelector((state) => state.settings.backUrl)

  useEffect((): (() => void) | undefined => {
    if (continuosMode) {
      const interval = setInterval(requestCostmap, 3000)
      return () => {
        clearInterval(interval)
      }
    }
    return
  }, [continuosMode])

  useEffect((): (() => void) | undefined => {
    if (wsStatus === ConnectionStatus.Connected) {
      const socket = new WebSocket(`ws://${backURL}/ws/costmap`)

      socket.onmessage = (e: MessageEvent): void => {
        const costmap = JSON.parse(e.data)
        setData(costmap)
      }

      socket.onclose = (): void => {
        console.log('costmap ws closed')
        setData(unknownCostmapValues)
      }

      socket.onopen = (): void => {
        console.log('costmap ws opened')
      }

      socket.onerror = (e: Event): void => {
        console.log(e)
      }

      return () => {
        socket.close()
      }
    }
    if (wsStatus === ConnectionStatus.Disconnected) {
      setData(unknownCostmapValues)
    }
    return
  }, [backURL, wsStatus])

  const mapDiv = (
    <div
      style={{
        height: 'calc(100% - 130px)',
        margin: '2%',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'stretch',
      }}
    >
      {data.map((rowValues, rowId) => Row(rowValues, rowId))}
    </div>
  )

  return (
    <Wrapper>
      <div style={{ height: '100%' }}>
        <h3>Kalman Live Feed</h3>
        {mapDiv}
        <div
          style={{
            display: 'flex',
          }}
        >
          {infoDiv(cellColors.UNKNOWN, 'No data')}
          {infoDiv(cellColors.EMPTY, 'Empty')}
          {infoDiv(cellColors.OBSTACLE, 'Obstacle')}
        </div>
        <button style={{ margin: '2%' }} onClick={requestCostmap}>
          Request costmap
        </button>
        Auto requests: {continuosMode ? 'On' : 'Off'}
        <button style={{ margin: '2%' }} onClick={(): void => setContinuosMode(!continuosMode)}>
          Toggle
        </button>
      </div>
    </Wrapper>
  )
}
