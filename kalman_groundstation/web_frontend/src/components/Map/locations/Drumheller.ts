import { circle } from '@turf/turf'
import type { MapOptions, SourceSpecification } from 'maplibre-gl'

import { locations } from '../mapUtils'

const searchAndRescuePoints = {
  reactor: [-112.716560364, 51.453865051],
  debris: [
    [-112.716438293, 51.453140259],
    [-112.716186523, 51.453266144],
    [-112.716339111, 51.453479767],
    [-112.71660614, 51.453620911],
  ],
}

const POLYGON_RADIUS = 5

const debrisSource: SourceSpecification = {
  type: 'geojson',
  data: {
    type: 'FeatureCollection',
    features: searchAndRescuePoints.debris.map((debris) => {
      return circle(debris, POLYGON_RADIUS, { units: 'meters' })
    }),
  },
}

const reactorSource: SourceSpecification = {
  type: 'geojson',
  data: circle(searchAndRescuePoints.reactor, 3, { units: 'meters' }),
}

export const mapDataDrumheller: (ref: React.RefObject<HTMLDivElement>) => MapOptions = (
  ref: React.RefObject<HTMLDivElement>,
) => ({
  container: ref.current as HTMLElement,
  style: {
    version: 8,
    sources: {
      'raster-tiles': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/{z}/{y}/{x}.jpeg'],
        tileSize: 256,
        minzoom: 16,
        maxzoom: 18,
      },
      'debris-source': debrisSource,
      'reactor-source': reactorSource,
    },
    layers: [
      {
        id: 'simple-tiles',
        type: 'raster',
        source: 'raster-tiles',
        minzoom: 16,
        maxzoom: 19,
      },
      {
        id: 'debrisLayer',
        type: 'fill',
        source: 'debris-source',
        layout: {},
        paint: {
          'fill-color': '#FF0000',
          'fill-opacity': 0.6,
        },
      },
      {
        id: 'reactorLayer',
        type: 'fill',
        source: 'reactor-source',
        layout: {},
        paint: {
          'fill-color': '#FFFF00',
          'fill-opacity': 0.8,
        },
      },
    ],
  },
  center: locations.Drumheller,
  zoom: 17,
  minZoom: 16,
  maxZoom: 18,
})
