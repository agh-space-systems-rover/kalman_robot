import type { MapOptions, SourceSpecification } from 'maplibre-gl'

export const marsyardSource = (mapPath: string): SourceSpecification => {
  return {
    type: 'image',
    url: `http://localhost:3000/${mapPath}`,
    coordinates: [
      [19.919374047619, 50.0649708564356],
      [19.919930047619, 50.0649708564356],
      [19.919930047619, 50.0645703564356],
      [19.919374047619, 50.0645703564356],
    ],
  }
}

export const mapDataD1: (ref: React.RefObject<HTMLDivElement>) => MapOptions = (
  ref: React.RefObject<HTMLDivElement>,
) => ({
  container: ref.current as HTMLElement,
  style: {
    version: 8,
    sources: {
      marsyardCostmap: {
        type: 'image',
        url: 'http://localhost:3000/erc_marsyard.png',
        coordinates: [
          [19.91937405, 50.06497086],
          [19.92009685, 50.06497086],
          [19.92009685, 50.06450806],
          [19.91937405, 50.06450806],
        ],
      },
      marsyardGrayscale: {
        type: 'image',
        url: 'http://localhost:3000/erc_marsyard_costmap.png',
        coordinates: [
          [19.91937405, 50.06497086],
          [19.92009685, 50.06497086],
          [19.92009685, 50.06450806],
          [19.91937405, 50.06450806],
        ],
      },
      'raster-tiles': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/{z}/{y}/{x}.jpeg'],
        tileSize: 256,
        minzoom: 17,
        maxzoom: 20,
      },
      'zoom-in': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/zoomin.jpeg'],
        tileSize: 256,
        maxzoom: 17,
      },
      'zoom-out': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/zoomout.jpeg'],
        tileSize: 256,
        minzoom: 20,
      },
    },
    layers: [
      {
        id: 'simple-tiles',
        type: 'raster',
        source: 'raster-tiles',
        minzoom: 17,
        maxzoom: 20,
      },
      {
        id: 'zoomin',
        type: 'raster',
        source: 'zoom-in',
        maxzoom: 17,
      },
      {
        id: 'zoomout',
        type: 'raster',
        source: 'zoom-out',
        minzoom: 20,
      },
      {
        id: 'marsyard-layer-costmap',
        type: 'raster',
        source: 'marsyardCostmap',
        paint: {
          'raster-fade-duration': 0,
        },
      },
      {
        id: 'marsyard-layer-grayscale',
        type: 'raster',
        source: 'marsyardGrayscale',
        paint: {
          'raster-fade-duration': 0,
        },
      },
    ],
  },
  center: { lon: 19.9196, lat: 50.0648 },
  // center: { lon: 77.537562, lat: 13.171052 },
  zoom: 19.5,
})
