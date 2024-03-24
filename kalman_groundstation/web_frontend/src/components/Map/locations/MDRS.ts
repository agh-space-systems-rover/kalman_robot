import type { MapOptions } from 'maplibre-gl'

export const mapDataMDRS: (ref: React.RefObject<HTMLDivElement>) => MapOptions = (
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
        minzoom: 15,
        maxzoom: 19,
      },
      'zoom-in': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/zoomin.jpeg'],
        tileSize: 256,
        maxzoom: 15,
      },
      'zoom-out': {
        type: 'raster',
        tiles: ['http://localhost:8081/tiles/zoomout.jpeg'],
        tileSize: 256,
        minzoom: 19,
      },
    },
    layers: [
      {
        id: 'simple-tiles',
        type: 'raster',
        source: 'raster-tiles',
        minzoom: 15,
        maxzoom: 19,
      },
      {
        id: 'zoomin',
        type: 'raster',
        source: 'zoom-in',
        maxzoom: 15,
      },
      {
        id: 'zoomout',
        type: 'raster',
        source: 'zoom-out',
        minzoom: 19,
      },
    ],
  },
  center: { lon: -76.54, lat: 39.18 },
  zoom: 17,
})
