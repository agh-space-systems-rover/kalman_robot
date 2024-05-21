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
    transform: rotate(${angle}deg);
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

export const RoverDisplay: React.FC<Wheels> = ({ targetMotors, motors }) => {
  return (
    <Rover>
      <WheelDisplay
        wheelId={WheelId.fl}
        displayType={DisplayType.Actual}
        velocity={motors.fl.velocity}
        angle={motors.fl.angle}
      />
      <WheelDisplay
        wheelId={WheelId.fl}
        displayType={DisplayType.Target}
        velocity={targetMotors.fl.velocity}
        angle={targetMotors.fl.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.br}
        displayType={DisplayType.Actual}
        velocity={motors.br.velocity}
        angle={motors.br.angle}
      />
      <WheelDisplay
        wheelId={WheelId.br}
        displayType={DisplayType.Target}
        velocity={targetMotors.br.velocity}
        angle={targetMotors.br.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.bl}
        displayType={DisplayType.Actual}
        velocity={motors.bl.velocity}
        angle={motors.bl.angle}
      />
      <WheelDisplay
        wheelId={WheelId.bl}
        displayType={DisplayType.Target}
        velocity={targetMotors.bl.velocity}
        angle={targetMotors.bl.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>

      <WheelDisplay
        wheelId={WheelId.fr}
        displayType={DisplayType.Actual}
        velocity={motors.fr.velocity}
        angle={motors.fr.angle}
      />
      <WheelDisplay
        wheelId={WheelId.fr}
        displayType={DisplayType.Target}
        velocity={targetMotors.fr.velocity}
        angle={targetMotors.fr.angle}
      >
        <SpeedIndicator />
      </WheelDisplay>
    </Rover>
  )
}
