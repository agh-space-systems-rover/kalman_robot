import 'maplibre-gl/dist/maplibre-gl.css'
import './map.css'

import type { Map as MapType, Marker } from 'maplibre-gl'
import maplibregl from 'maplibre-gl'
import React, { useEffect, useRef, useState } from 'react'

import { useAppSelector } from '../../store/storeHooks'
import { useBleSignals } from './hooks/useBleSignals'
import { useGpsPath } from './hooks/useGpsPath'
import { useRulerTool } from './hooks/useRulerTool'
import { useUserMarkers } from './hooks/useUserMarkers'
// import { mapDataDrumheller } from './locations/Drumheller'
import { mapDataD1 } from './locations/D1'
import { MapControlPanel } from './MapControlPanel'
import { locations, mapJumpTo } from './mapUtils'

export interface Markers {
  rover: Marker
  goal: Marker
  tags: Marker[]
  userMarkers: Marker[]
}

export interface MapSettings {
  followRover: boolean
  odometryMode: boolean
  showBleSignal: boolean
  showGpsPath: boolean
}

export const Map: () => JSX.Element = () => {
  const mapContainerRef = useRef<HTMLDivElement>(null)
  const [map, setMap] = React.useState<MapType>()

  const goalGpsPosition = useAppSelector((state) => ({
    // TODO FIX ME LATER, we need to flip it and fix supervisor control.py file
    lat: state.autonomy.usp?.x || 0,
    lon: state.autonomy.usp?.y || 0,
  }))

  const defaultRoverPosition = locations.Drumheller
  const roverGpsPostion = useAppSelector((state) => ({
    lat: state.autonomy.gps.position.latitude || defaultRoverPosition.lat,
    lon: state.autonomy.gps.position.longitude || defaultRoverPosition.lon,
  }))
  const roverOrientation = useAppSelector((state) => state.autonomy.imu.gamma)

  const [mapSettings, setMapSettings] = React.useState<MapSettings>({
    followRover: true,
    odometryMode: false,
    showBleSignal: false,
    showGpsPath: true,
  })

  const [markers, setMarkers] = useState<Markers>({
    rover: new maplibregl.Marker(),
    goal: new maplibregl.Marker(),
    tags: [],
    userMarkers: [],
  })

  // initial use effect
  useEffect(() => {
    // const newMap = new maplibregl.Map(mapDataDrumheller(mapContainerRef))
    const newMap = new maplibregl.Map(mapDataD1(mapContainerRef))
    newMap.on('drag', () => {
      setMapSettings((prevSettings) => ({ ...prevSettings, followRover: false }))
    })

    const markerElement = document.createElement('div')
    markerElement.style.width = '24px'
    markerElement.style.height = '24px'
    markerElement.style.backgroundImage = 'url("http://localhost:8081/tiles/arrow-24.png")'

    const roverMarker = new maplibregl.Marker({ element: markerElement }).setLngLat(roverGpsPostion).addTo(newMap)
    setMarkers((prevMarkers) => {
      return { ...prevMarkers, rover: roverMarker }
    })

    const goalMarker = new maplibregl.Marker({ color: '#00FF00' }).setLngLat(goalGpsPosition).addTo(newMap)
    setMarkers((prevMarkers) => {
      return { ...prevMarkers, goal: goalMarker }
    })

    setMap(newMap)

    return () => {
      map?.remove()
    }
  }, [])

  // TODO refactor into one useUpdateMarkers hook
  useEffect(() => {
    markers.rover?.setLngLat(roverGpsPostion)

    if (roverOrientation) {
      markers.rover?.setRotation(-((roverOrientation / 3.14) * 180))
    }

    if (mapSettings.followRover && map) {
      mapJumpTo(map, roverGpsPostion)
    }
  }, [roverGpsPostion, roverOrientation, mapSettings.followRover])

  useEffect(() => {
    markers.goal?.setLngLat(goalGpsPosition)
  }, [goalGpsPosition])

  const controlPanel = map ? (
    <MapControlPanel
      map={map}
      goalGpsPosition={goalGpsPosition}
      markers={markers}
      mapSettings={mapSettings}
      setMapSettings={setMapSettings}
    />
  ) : null
  useBleSignals(map, mapSettings.showBleSignal)
  useGpsPath(map, mapSettings.showGpsPath)
  useRulerTool(map)
  useUserMarkers(map, markers, setMarkers)

  return (
    <div className='mapWrapper'>
      <div ref={mapContainerRef} style={{ gridArea: 'mapdisplay' }}></div>
      {controlPanel}
    </div>
  )
}
