import styled from 'styled-components'

import { requestProbeMeasurement, setRawDigitalOutput } from '../../api/requests'
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
      <h6>Probe</h6>
      <button onClick={requestProbeMeasurement}>Request probe</button>
      <div style={{ fontSize: 16 }}>Temperature: {smartProbe.temperature.toFixed(2)}C</div>
      <div style={{ fontSize: 16 }}>Humidity: {smartProbe.hummidity.toFixed(2)}%</div>

      <h6>Sample1</h6>
      <button onClick={(): void => setRawDigitalOutput(0, 180)}>Open</button>
      <button onClick={(): void => setRawDigitalOutput(0, 0)}>Close</button>
      <h6>Sample2</h6>
      <button onClick={(): void => setRawDigitalOutput(1, 180)}>Open</button>
      <button onClick={(): void => setRawDigitalOutput(1, 0)}>Close</button>
    </ColumnWrapper>
  )
}
