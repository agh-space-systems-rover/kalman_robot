import styled from 'styled-components'

import { resetServoServer } from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'
import { ArmPositioning } from './ArmPositioning'

const Wrapper = styled.div`
  grid-area: arm;
  & h5 {
    font-size: 48px;
  }
  text-align: center;
  overflow: scroll;
  background-repeat: no-repeat;
`

export const Arm: () => JSX.Element = () => {
  const armState = useAppSelector((state) => state.arm)
  const jointsState = armState.jointPositions
  const jointLimits = armState.jointLimits

  const resetServoCallback: () => void = () => {
    resetServoServer()
  }

  return (
    <Wrapper>
      <h5>Arm</h5>
      <div>Arm Mode: {armState.armMode} </div>
      Positions:
      {jointsState.joints.map((joint, idx) => (
        <div
          key={idx}
          style={{
            color: `${joint > jointLimits[idx].lower && joint < jointLimits[idx].upper ? '#000' : '#f00'}`,
          }}
        >
          joint {idx + 1}: {((joint / 3.1415) * 180).toFixed(0)}
        </div>
      ))}
      <div>gripper: {((jointsState.gripper / 3.1415) * 180).toFixed(0)}</div>
      <div>state: {jointsState.status}</div>
      <div>collisionVelocityFactor: {jointsState.collisionVelocityFactor}</div>
      <div>
        <button onClick={resetServoCallback}>Reset servo_server</button>
      </div>
      <ArmPositioning />
    </Wrapper>
  )
}
