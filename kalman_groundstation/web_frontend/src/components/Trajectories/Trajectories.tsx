import { useEffect } from 'react'
import styled from 'styled-components'

import { executeTrajectory, getTrajectories, planTrajectory } from '../../api/requests'
import { ConnectionStatus } from '../../features/websocket/websocketTypes'
import { useAppSelector } from '../../store/storeHooks'
import { selectTrajectories, selectTrajectoriesRequestState } from '../../store/Trajectories/trajectoriesSlice'
import type { Trajectory } from '../../store/Trajectories/trajectoriesTypes'
import { TrajectoryType } from '../../store/Trajectories/trajectoriesTypes'
import { selectWebsocket } from '../Websocket/websocketSlice'
import { NewTrajectories } from './NewTrajectories'

const Wrapper = styled.div`
  grid-area: trajectories;
  overflow: scroll;
`

const TrajectoryUI: (trajectory: Trajectory, id: number, type: TrajectoryType) => JSX.Element = (
  trajectory: Trajectory,
  id: number,
  type: TrajectoryType,
) => {
  return (
    <div key={id}>
      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        {trajectory.label}
        <div>
          <button onClick={(): void => planTrajectory(trajectory, type)}>Plan</button>
          <button onClick={(): void => executeTrajectory(trajectory, type)}>Execute</button>
        </div>
      </div>
    </div>
  )
}

export const Trajectories: () => JSX.Element = () => {
  const trajectories = useAppSelector(selectTrajectories)
  const requestState = useAppSelector(selectTrajectoriesRequestState)
  const websocket = useAppSelector(selectWebsocket)

  useEffect(() => {
    if (websocket.connectionStatus === ConnectionStatus.Connected) {
      getTrajectories()
    }
  }, [websocket, requestState])

  const uiCartesian = trajectories.cartesianSpaceGoals?.map((t, i) => TrajectoryUI(t, i, TrajectoryType.Cartesian))

  return (
    <Wrapper>
      <h3>Trajectories</h3>
      {/* <h5>Joints:</h5>
      {ui_joints} */}
      <h5>Cartesian:</h5>
      {uiCartesian}
      <NewTrajectories />
    </Wrapper>
  )
}
