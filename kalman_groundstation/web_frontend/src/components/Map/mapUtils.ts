import * as _ from 'lodash'
import type { Map as MapType } from 'maplibre-gl'

export declare class GeoJSONFeature {
  type: 'Feature'
  geometry: GeoJSON.Point | GeoJSON.LineString
  properties: {
    [name: string]: any
  }
}

export const locations = {
  Drumheller: { lon: -112.715501, lat: 51.452773 },
  Bangalore: { lon: 77.536404, lat: 13.170847 },
  GreenRiver: { lon: -110.139383, lat: 38.99218 },
  MDRS: { lon: -110.791683, lat: 38.406359 },
  D1: { lon: 19.919152, lat: 50.064588 },
}

export const mapJumpTo: (map: MapType, position: { lat: number; lon: number }) => void = (map, position) => {
  map?.jumpTo({
    center: position,
    zoom: 18,
  })
}

export const getMarkerHue: (values: number[], idx: number) => number = (values, idx) => {
  const max = _.max(values)
  const min = _.min(values)
  if (!min || !max) {
    throw new Error('Cannot calculate Hue from empty or falsey array')
  }
  const hue = 240 + 120 * ((values[idx] - min) / (max - min))
  return hue
}
