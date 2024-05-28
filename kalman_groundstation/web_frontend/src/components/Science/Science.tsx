import type React from 'react'
import styled from 'styled-components'

// import { ERCScience } from './ERC/ERCScience'
import { ERCScience } from './URC2024/ERCScience'

const Wrapper = styled.div`
  grid-area: science;
`

export const Science: React.FC = () => {
  return (
    <Wrapper>
      <ERCScience />
    </Wrapper>
  )
}
