import * as _ from 'lodash'
import { type GeoJSONSource, type Map as MapType } from 'maplibre-gl'
import { useCallback, useEffect, useState } from 'react'

import type { RootState } from '../../../store/store'
import { useAppSelector } from '../../../store/storeHooks'
import { type GeoJSONFeature, getMarkerHue } from '../mapUtils'

export const useGpsPath: (map: MapType | undefined, show: boolean) => void = (map, show) => {
  const throttleSetter = useCallback(
    _.throttle(
      (state: RootState) => ({
        lat: state.autonomy.gps.position.latitude,
        lon: state.autonomy.gps.position.longitude,
        alt: state.autonomy.gps.position.altitude,
      }),
      1000,
    ),
    [],
  )
  const roverGpsPostion: { lat: number | null; lon: number | null; alt: number | null } | undefined =
    useAppSelector(throttleSetter)
  const [features, setFeatures] = useState<GeoJSONFeature[]>([])
  const [updateCounter, setUpdateCounter] = useState<number>(0)

  useEffect(() => {
    if (!map?.getLayer('gpsPoints')) {
      map?.addSource('gpsPath', {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: features },
      })
      map?.addLayer({
        id: 'gpsPoints',
        type: 'circle',
        source: 'gpsPath',
        layout: {
          visibility: 'visible',
        },
        paint: {
          'circle-radius': 4,
          'circle-color': ['get', 'color'],
        },
      })
    } else {
      if (show) {
        map?.getLayer('gpsPoints')?.setLayoutProperty('visibility', 'visible')
      } else {
        map?.getLayer('gpsPoints').setLayoutProperty('visibility', 'none')
      }
    }
  }, [map, show])

  useEffect(() => {
    if (roverGpsPostion) {
      const { lat, lon, alt } = roverGpsPostion
      if (lat && lon && alt) {
        setUpdateCounter((count) => count + 1)
        const newFeature: GeoJSONFeature = {
          type: 'Feature',
          geometry: {
            type: 'Point',
            coordinates: [lon, lat],
          },
          properties: { color: 'hsl(240, 100%, 50%)', alt: alt },
        }
        setFeatures((prevFeatures: GeoJSONFeature[]) => {
          let newFeatures = [...prevFeatures, newFeature]
          newFeatures = newFeatures.map((feature, idx) => {
            const newFeature = feature
            const hue = getMarkerHue(
              newFeatures.map((feature) => feature.properties.alt),
              idx,
            )
            const newColor = `hsl(${hue},100%, 50%)`
            newFeature.properties.color = newColor
            return newFeature
          })
          return newFeatures
        })
      }
    }
  }, [roverGpsPostion])

  useEffect(() => {
    if (updateCounter % 10 == 0) {
      const source: GeoJSONSource | undefined = map?.getSource('gpsPath') as GeoJSONSource
      if (source) {
        source.setData({ type: 'FeatureCollection', features: features })
      }
    }
  }, [features, updateCounter])
}
