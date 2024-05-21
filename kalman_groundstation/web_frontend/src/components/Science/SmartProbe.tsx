import styled from 'styled-components'

import { requestProbeMeasurement } from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'

const ColumnWrapper = styled.div`
  padding: 4px;
  display: flex;
  flex-direction: column;
  gap: 8px;
  text-align: center;
  & > h5 {
    font-size: 24px;
  }
  border: 1px solid black;
  & > button {
    border: 1px solid black;
  }
  & > button:hover {
    background-color: #ccc;
  }
  & > button:active {
    background-color: #aaa;
  }
`

export const SmartProbe: () => JSX.Element = () => {
  const smartProbe = useAppSelector((state) => state.science.smartProbe)

  return (
    <ColumnWrapper>
      <h5>Probe</h5>
      <button onClick={requestProbeMeasurement}>Request probe</button>
      <div style={{ fontSize: 16 }}>Temperature: {smartProbe.temperature.toFixed(2)}C</div>
      <div style={{ fontSize: 16 }}>Humidity: {smartProbe.hummidity.toFixed(2)}%</div>
    </ColumnWrapper>
  )
}
