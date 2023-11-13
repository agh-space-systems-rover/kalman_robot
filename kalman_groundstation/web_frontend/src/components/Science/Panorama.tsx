import styled from 'styled-components'

import { requestLampPWM, requestPanorama } from '../../api/requests'
import { PanoramaStatus } from '../../store/Science/scienceTypes'
import { useAppSelector } from '../../store/storeHooks'
import { locations } from '../Map/mapUtils'

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

export const Panorama: () => JSX.Element = () => {
  const panoramaStatus = useAppSelector((state) => state.science.panoramaStatus)
  const defaultRoverPosition = locations.Drumheller
  const roverGpsPostion = useAppSelector((state) => ({
    lat: state.autonomy.gps.position.latitude || defaultRoverPosition.lat,
    lon: state.autonomy.gps.position.longitude || defaultRoverPosition.lon,
    alt: state.autonomy.gps.position.altitude || 0,
  }))

  const getPanoramaText: () => string = () => {
    if (panoramaStatus == PanoramaStatus.INITIAL) {
      return 'Nothing sent yet.'
    } else if (panoramaStatus == PanoramaStatus.WAITING) {
      return 'Sent and waiting.'
    } else if (panoramaStatus == PanoramaStatus.SUCCES) {
      return 'Succes!'
    } else if (panoramaStatus == PanoramaStatus.FAILURE) {
      return 'Failure!'
    } else {
      return 'Code error!'
    }
  }

  return (
    <ColumnWrapper>
      <h5>Panorama</h5>
      <button onClick={(): void => requestPanorama(roverGpsPostion.lat, roverGpsPostion.lon, roverGpsPostion.alt)}>
        Take a photo
      </button>
      <div style={{ fontSize: 16 }}>{getPanoramaText()}</div>
      <button onClick={(): void => requestLampPWM(100)}>LAMP ON</button>
      <button onClick={(): void => requestLampPWM(0)}>LAMP OFF</button>
    </ColumnWrapper>
  )
}
