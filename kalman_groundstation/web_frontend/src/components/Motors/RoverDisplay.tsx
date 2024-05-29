import type React from 'react'
import type { FlattenSimpleInterpolation } from 'styled-components'
import styled, { css } from 'styled-components'

import type { Wheel, Wheels } from '../../store/Motors/motorTypes'

const Rover = styled.div`
  width: 150px;
  height: 200px;
  margin: 50px auto;
  background-color: gray;
  position: relative;
`

interface WheelDisplayProps extends Wheel {
  wheelId: WheelId
  displayType: DisplayType
}

enum DisplayType {
  Target = 'Target',
  Actual = 'Actual',
}

enum WheelId {
  fl = 'fl',
  bl = 'bl',
  br = 'br',
  fr = 'fr',
}

const getWheelPosition: (wheelId: WheelId) => FlattenSimpleInterpolation = (wheelId: WheelId) => {
  switch (wheelId) {
    case WheelId.fl:
      return css`
        top: -10px;
        left: -25px;
      `
    case WheelId.bl:
      return css`
        bottom: -10px;
        left: -25px;
      `
    case WheelId.br:
      return css`
        bottom: -10px;
        right: -25px;
      `
    case WheelId.fr:
      return css`
        top: -10px;
        right: -25px;
      `
  }
}

const WheelDisplay = styled.div<WheelDisplayProps>`
  width: 50px;
  height: 70px;
  ${({ displayType }): string => (displayType === DisplayType.Actual ? 'background-color: red; opacity: 60%;' : '')}
  border: black 2px solid;
  position: absolute;
  ${({ wheelId }): FlattenSimpleInterpolation => getWheelPosition(wheelId)}
  ${({ velocity, angle }): FlattenSimpleInterpolation => css`
    transform: rotate(${3.14 - angle}rad);
    & > div {
      top: calc(35px - 2.5px - ${(velocity * (35 - 2.5)) / 100}px);
    }
  `}
`

const SpeedIndicator = styled.div`
  width: 46px;
  height: 5px;
  background-color: black;
  position: absolute;
`

/* eslint-disable camelcase */
export const RoverDisplay: React.FC<Wheels> = ({ target_motors, motors }) => {
  return (
    <Rover>
      <WheelDisplay
        wheelId={WheelId.fl}
        displayType={DisplayType.Actual}
        velocity={motors.front_left.velocity}
        angle={motors.front_left.angle}
      />
      <WheelDisplay
        wheelId={WheelId.fl}
        displayType={DisplayType.Target}
        velocity={target_motors.front_left.velocity}
        angle={target_motors.front_left.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.br}
        displayType={DisplayType.Actual}
        velocity={motors.back_right.velocity}
        angle={motors.back_right.angle}
      />
      <WheelDisplay
        wheelId={WheelId.br}
        displayType={DisplayType.Target}
        velocity={target_motors.back_right.velocity}
        angle={target_motors.back_right.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.bl}
        displayType={DisplayType.Actual}
        velocity={motors.back_left.velocity}
        angle={motors.back_left.angle}
      />
      <WheelDisplay
        wheelId={WheelId.bl}
        displayType={DisplayType.Target}
        velocity={target_motors.back_left.velocity}
        angle={target_motors.back_left.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.fr}
        displayType={DisplayType.Actual}
        velocity={motors.front_right.velocity}
        angle={motors.front_right.angle}
      />
      <WheelDisplay
        wheelId={WheelId.fr}
        displayType={DisplayType.Target}
        velocity={target_motors.front_right.velocity}
        angle={target_motors.front_right.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>
    </Rover>
  )
}
