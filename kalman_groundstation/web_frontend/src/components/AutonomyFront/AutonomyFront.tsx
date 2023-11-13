import styled from 'styled-components'

import { AutonomyAzimuth } from '../AutonomyAzimuth/AutonomyAzimuth'
import { AutonomyMonitor } from './AutonomyMonitor'
import { Usp } from './Usp'

const Wrapper = styled.div`
  grid-area: autonomy;
  display: grid;
  grid-template-areas: 'dynamic' 'monitor';
  grid-template-rows: 2fr 1fr;
  grid-template-columns: 1fr;
`

export const AutonomyFront: () => JSX.Element = () => {
  return (
    <Wrapper>
      <Usp />
      <AutonomyMonitor />
      <AutonomyAzimuth />
    </Wrapper>
  )
}
