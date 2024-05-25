import * as _ from 'lodash'
import type { Map as MapType } from 'maplibre-gl'
import maplibregl from 'maplibre-gl'

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

// Interface definitions
interface GpsCoordinates {
  label: string
  latitude: number
  longitude: number
}

// Function to parse the GPS coordinates
function parseGpsCoordinates(input: string): GpsCoordinates[] {
  const lines = input.split('\n')
  const coordinates: GpsCoordinates[] = []

  lines.forEach((line) => {
    const parts = line.trim().split(' ')
    const label = parts[0]

    if (parts.length === 3) {
      const latitude = parseFloat(parts[1])
      const longitude = parseFloat(parts[2])
      coordinates.push({ label, latitude, longitude })
    } else if (parts.length === 9) {
      const directionLat = parts[1]
      const degreesLat = parseInt(parts[2])
      const minutesLat = parseInt(parts[3])
      const secondsLat = parseInt(parts[4])
      const directionLon = parts[5]
      const degreesLon = parseInt(parts[6])
      const minutesLon = parseInt(parts[7])
      const secondsLon = parseInt(parts[8])

      const latitude = dmsToDecimal(directionLat, degreesLat, minutesLat, secondsLat)
      const longitude = dmsToDecimal(directionLon, degreesLon, minutesLon, secondsLon)
      coordinates.push({ label, latitude, longitude })
    } else {
      alert('unable to parse: ' + parts.toString())
    }
  })

  return coordinates
}

// Function to convert DMS to Decimal
function dmsToDecimal(direction: string, degrees: number, minutes: number, seconds: number): number {
  let decimal = degrees + minutes / 60 + seconds / 3600
  if (direction === 'S' || direction === 'W') {
    decimal = -decimal
  }
  return decimal
}

export const loadWaypoints: (map: MapType) => void = (map) => {
  const textarea = document.getElementById('gps-waypoints-data') as HTMLInputElement
  // console.log(textarea.value)
  // console.log(map)
  const coordinates = parseGpsCoordinates(textarea.value)
  coordinates.forEach((coord) => {
    console.log(coord)
    new maplibregl.Marker()
      .setLngLat([coord.longitude, coord.latitude])
      .setPopup(new maplibregl.Popup().setText(coord.label)) // Add popups
      .addTo(map)
  })
}

// waypoint1 51.45302204448234 -112.71581991471845
// waypoint2 51.453090573262614 -112.71610959329223
// waypoint3 N 51 27 11 W 112 42 59
