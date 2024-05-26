import 'maplibre-gl/dist/maplibre-gl.css'
import './map.css'

import type { Map as MapType } from 'maplibre-gl'
import type React from 'react'
import { useState } from 'react'

import { logUserMark } from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'
import type { MapSettings, Markers } from './Map'
import { loadWaypoints, mapJumpTo } from './mapUtils'

interface Props {
  map: MapType
  goalGpsPosition: { lat: number; lon: number }
  markers: Markers
  mapSettings: MapSettings
  setMapSettings: React.Dispatch<React.SetStateAction<MapSettings>>
}

export const MapControlPanel: React.FC<Props> = ({ map, goalGpsPosition, markers, mapSettings, setMapSettings }) => {
  const bleSignal = useAppSelector((state) => state.autonomy.bleSignal)

  const [id, setId] = useState<number>(0)
  const [desc, setDesc] = useState<string>('opis')

  return (
    <div className='controlPanelWrapper'>
      <button
        onClick={(): void => {
          setMapSettings((prevSettings) => ({ ...prevSettings, followRover: true }))
        }}
      >
        Follow rover
      </button>
      <button
        onClick={(): void => {
          setMapSettings((prevSettings) => ({ ...prevSettings, followRover: false }))
          mapJumpTo(map, goalGpsPosition)
        }}
      >
        Jump to goal
      </button>
      {/* ############## CIRC only feature ############## */}
      <div style={{ display: 'none' }}>
        <span style={{ marginLeft: 3 }}>BLE Signal: {bleSignal}</span>
        <br />
        <input
          id='show-ble'
          type='checkbox'
          checked={mapSettings.showBleSignal}
          onChange={(): void => {
            setMapSettings((prevSettings) => ({ ...prevSettings, showBleSignal: !prevSettings.showBleSignal }))
          }}
        />
        <label htmlFor='show-ble'>Show BLE signal</label>
      </div>
      {/* ############################################## */}
      <input
        id='show-gps-path'
        type='checkbox'
        checked={mapSettings.showGpsPath}
        onChange={(): void => {
          setMapSettings((prevSettings) => ({ ...prevSettings, showGpsPath: !prevSettings.showGpsPath }))
        }}
      />
      <label htmlFor='show-gps-path'>Show GPS Path</label>
      <div style={{ display: 'none' }}>
        <br />
        <input id='user-marker-id' type='number' value={id} onChange={(e): void => setId(parseInt(e.target.value))} />
        <label htmlFor='show-gps-path'>UM ID </label>
        <input id='user-marker-desc' type='text' value={desc} onChange={(e): void => setDesc(e.target.value)} />
        <label htmlFor='show-gps-path'>UM Desc</label>
        <br />
        <button
          onClick={(): void => {
            logUserMark(markers.userMarkers[id].getLngLat().lat, markers.userMarkers[id].getLngLat().lng, 0, desc)
          }}
        >
          Save point!
        </button>
      </div>

      <button
        onClick={(): void => {
          loadWaypoints(map)
        }}
      >
        Load waypoints
      </button>
    </div>
  )
}
