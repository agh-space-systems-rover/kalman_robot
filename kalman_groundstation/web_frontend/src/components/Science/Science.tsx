import type React from 'react'
import styled from 'styled-components'

import { ERCScience } from './ERC/ERCScience'

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
