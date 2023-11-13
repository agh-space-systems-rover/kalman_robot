import * as _ from 'lodash'
import { useCallback } from 'react'
import styled from 'styled-components'

import { initialWheelsState } from '../../store/Motors/motorsSlice'
import type { Wheels } from '../../store/Motors/motorTypes'
import { useAppSelector } from '../../store/storeHooks'
import { RoverDisplay } from './RoverDisplay'
import { Temperatures } from './Temperatures'

const Wrapper = styled.div`
  grid-area: motors;
  padding: 4px;
  display: flex;
  flex-direction: column;
  justify-content: space-evenly;
  gap: 8px;
  text-align: center;
  overflow: hidden;

  & > h5 {
    font-size: 24px;
  }
`

export const Motors: () => JSX.Element = () => {
  const throttleSetter = useCallback(
    _.throttle((state: any): Wheels => state.motors.wheels, 200),
    [],
  )
  const drivingMode = useAppSelector((state) => state.motors.settings.drivingMode)
  const wheelsStatus = useAppSelector(throttleSetter) || initialWheelsState.wheels
  const wheelsScalingFactor = useAppSelector((state) => state.motors.settings.scalingFactor)

  return (
    <Wrapper>
      <h5>{drivingMode}</h5>
      <RoverDisplay targetMotors={wheelsStatus.targetMotors} motors={wheelsStatus.motors} />
      <h5>
        ⬆️ {wheelsScalingFactor.forward.toFixed(2)} ⬆️
        <br />
        ⬅️ {wheelsScalingFactor.turn.toFixed(2)} ➡️
      </h5>
      <Temperatures />
    </Wrapper>
  )
}
