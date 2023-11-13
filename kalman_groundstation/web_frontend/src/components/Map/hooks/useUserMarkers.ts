import * as _ from 'lodash'
import type { MapMouseEvent } from 'maplibre-gl'
import maplibregl, { type Map as MapType } from 'maplibre-gl'
import type React from 'react'
import { useCallback, useEffect } from 'react'

import type { RootState } from '../../../store/store'
import { useAppSelector } from '../../../store/storeHooks'
import type { Markers } from '../Map'

export const useUserMarkers: (
  map: MapType | undefined,
  markers: Markers,
  setMarkers: React.Dispatch<React.SetStateAction<Markers>>,
) => void = (map, markers, setMarkers) => {
  const throttleSetter = useCallback(
    _.throttle((state: RootState) => state.autonomy.gps.position.altitude, 1000),
    [],
  )
  const altitude = useAppSelector(throttleSetter) || 1 // FIX ME LATER, 1 shouldnt be here probably
  const onDoubleClick = useCallback(
    (e: MapMouseEvent) => {
      e.preventDefault()
      if (map && altitude) {
        console.log('add marker')
        const popup = new maplibregl.Popup({ offset: 25 }).setHTML(
          `
          <div class='userMarker'>
            <h5>ID: ${markers.userMarkers.length}</h5>
            <h5>lat: ${e.lngLat.lat}</h5>
            <h5>lon: ${e.lngLat.lng}</h5>
          </div>
        `,
        )

        const newMarker = new maplibregl.Marker({ draggable: true })
          .setLngLat({
            lon: e.lngLat.lng,
            lat: e.lngLat.lat,
          })
          .addTo(map)

        newMarker.on('dragend', (event) => {
          const popup = new maplibregl.Popup({ offset: 25 }).setHTML(
            `
          <div class='userMarker'>
            <h5>lat: ${event.target.getLngLat().lat}, lon: ${event.target.getLngLat().lng}<h5>
          </div>
        `,
          )
          event.target.setPopup(popup)
        })
        newMarker.setPopup(popup)
        setMarkers((prevMarkers) => ({ ...prevMarkers, userMarkers: [...prevMarkers.userMarkers, newMarker] }))
      }
    },
    [map, markers.userMarkers.length, altitude, setMarkers],
  )

  useEffect(() => {
    map?.on('dblclick', onDoubleClick)

    return () => {
      map?.off('dblclick', onDoubleClick)
    }
  }, [map, onDoubleClick])
}
