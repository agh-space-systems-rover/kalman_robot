import { length } from '@turf/turf'
import * as _ from 'lodash'
import type { GeoJSONSource, MapMouseEvent, Popup } from 'maplibre-gl'
import maplibregl, { type Map as MapType } from 'maplibre-gl'
import { useCallback, useEffect, useState } from 'react'

import { type GeoJSONFeature } from '../mapUtils'

export const useRulerTool: (map: MapType | undefined) => void = (map) => {
  const [features, setFeatures] = useState<GeoJSONFeature[]>([])
  const [popup, setPopup] = useState<Popup | null>()

  const cleanRuler = useCallback(() => {
    setFeatures([])
    popup?.remove()
    setPopup(null)
  }, [popup])

  const onClick = useCallback(
    (e: MapMouseEvent) => {
      if (features.length >= 2 || !map) {
        cleanRuler()
        return
      }
      if (e.type != 'contextmenu') {
        return
      }

      const newFeatures: GeoJSONFeature[] = []
      const newPointFeature: GeoJSONFeature = {
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [e.lngLat.lng, e.lngLat.lat],
        },
        properties: {},
      }
      newFeatures.push(newPointFeature)

      if (features.length >= 1) {
        const coordinates: GeoJSON.Position[] = [
          features[0].geometry.coordinates as GeoJSON.Position,
          [e.lngLat.lng, e.lngLat.lat],
        ]
        const newLineFeature: GeoJSONFeature = {
          type: 'Feature',
          geometry: {
            type: 'LineString',
            coordinates: coordinates,
          },
          properties: {},
        }
        newFeatures.push(newLineFeature)

        const distance = (length(newLineFeature) * 1000).toFixed(1)
        const popup = new maplibregl.Popup({ closeOnClick: false })
          .setLngLat([e.lngLat.lng, e.lngLat.lat])
          .setHTML(`<h5>${distance}m</h5>`)
          .addTo(map)

        setPopup(popup)

        console.log(distance)
      }

      setFeatures((prevFeatures) => [...prevFeatures, ...newFeatures])
    },
    [features],
  )

  useEffect(() => {
    map?.on('click', onClick)
    map?.on('contextmenu', onClick)

    return () => {
      map?.off('click', onClick)
      map?.off('contextmenu', onClick)
    }
  }, [map, onClick, cleanRuler])

  useEffect(() => {
    if (!map?.getSource('rulerSource')) {
      map?.addSource('rulerSource', {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: [] },
      })
      map?.addLayer({
        id: 'rulerPoints',
        type: 'circle',
        source: 'rulerSource',
        paint: {
          'circle-radius': 5,
          'circle-color': '#000',
        },
        filter: ['in', '$type', 'Point'],
      })
      map?.addLayer({
        id: 'rulerLine',
        type: 'line',
        source: 'rulerSource',
        layout: {
          'line-cap': 'round',
          'line-join': 'round',
        },
        paint: {
          'line-color': '#000',
          'line-width': 2.5,
        },
        filter: ['in', '$type', 'LineString'],
      })
    }
  }, [map])

  useEffect(() => {
    const source: GeoJSONSource | undefined = map?.getSource('rulerSource') as GeoJSONSource
    if (source && features) {
      source.setData({ type: 'FeatureCollection', features: features })
    }
  }, [map, features])
}
