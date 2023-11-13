import type { MapOptions } from 'maplibre-gl'

export const mapDataGreenRiver: (ref: React.RefObject<HTMLDivElement>) => MapOptions = (
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
    ],
  },
  center: { lon: -110.140152, lat: 38.992725 },
  zoom: 18,
})
