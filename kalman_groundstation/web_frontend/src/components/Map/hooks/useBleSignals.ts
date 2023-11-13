import type { GeoJSONSource } from 'maplibre-gl'
import { type Map as MapType } from 'maplibre-gl'
import { useEffect, useState } from 'react'

import { useAppSelector } from '../../../store/storeHooks'
import { type GeoJSONFeature, getMarkerHue } from '../mapUtils'

export interface Signal {
  signalValue: number
  lon: number
  lat: number
}

const MAXIMUM_NUMBER_OF_MARKERS = 180

export const useBleSignals: (map: MapType | undefined, show: boolean) => void = (map, show) => {
  const bleSignal = useAppSelector((state) => state.autonomy.bleSignal)
  const roverGpsPostion = useAppSelector((state) => ({
    lat: state.autonomy.gps.position.latitude,
    lon: state.autonomy.gps.position.longitude,
    alt: state.autonomy.gps.position.altitude,
  }))
  const [signals, setSignals] = useState<Signal[]>([])
  const [features, setFeatures] = useState<GeoJSONFeature[]>([])

  useEffect(() => {
    if (!map?.getLayer('bleSignals')) {
      map?.addSource('signals', {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: features },
      })
      map?.addLayer({
        id: 'bleSignals',
        type: 'circle',
        source: 'signals',
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
        map?.getLayer('bleSignals')?.setLayoutProperty('visibility', 'visible')
      } else {
        map?.getLayer('bleSignals').setLayoutProperty('visibility', 'none')
      }
    }
  }, [map, show])

  useEffect(() => {
    const { lat, lon } = roverGpsPostion
    if (bleSignal && lat && lon) {
      if (signals.length > MAXIMUM_NUMBER_OF_MARKERS) {
        setSignals((prevSignals) => prevSignals.filter((signal, idx) => idx != 0))
        setFeatures((prevFeatures) => prevFeatures.filter((feature, idx) => idx != 0))
      }

      const newFeature: GeoJSONFeature = {
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [lon, lat],
        },
        properties: { color: 'hsl(240, 100%, 50%)' },
      }

      setFeatures((prevFeatures) => [...prevFeatures, newFeature])

      setSignals((prevSignals) => {
        return [...prevSignals, { signalValue: bleSignal, lat: lat, lon: lon }]
      })
    }
  }, [bleSignal])

  useEffect(() => {
    const source: GeoJSONSource | undefined = map?.getSource('signals') as GeoJSONSource
    if (source) {
      source.setData({ type: 'FeatureCollection', features: features })
    }
  }, [features])

  useEffect(() => {
    setFeatures((prevFeatures) =>
      prevFeatures.map((feature, idx) => {
        const newFeature = feature
        const hue = getMarkerHue(
          signals.map((signal) => signal.signalValue),
          idx,
        )
        const newColor = `hsl(${hue},100%, 50%)`
        newFeature.properties.color = newColor
        return newFeature
      }),
    )
  }, [signals])
}
