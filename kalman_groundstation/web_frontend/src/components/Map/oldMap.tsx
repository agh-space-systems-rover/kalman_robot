import 'maplibre-gl/dist/maplibre-gl.css'
import './map.css'

import type { AxiosResponse } from 'axios'
import type { Map as MapType, Marker } from 'maplibre-gl'
import maplibregl from 'maplibre-gl'
import React, { useEffect, useRef, useState } from 'react'
import styled from 'styled-components'

import { getRequest } from '../../api/axios'
import { useAppSelector } from '../../store/storeHooks'
import { mapDataD1 } from './locations/D1'
// import { mapDataGreenRiver } from './locations/GreenRiver'

enum ImagePaths {
  GRAYSCALE = 'marsyard_grayscale.png',
  STATIC_COSTMAP = 'erc_marsyard.png',
  CONE = 'cone.png',
  ARROW = 'arrow.png',
}

const Wrapper = styled.div`
  height: 100%;
  grid-area: map;
  display: grid;
  grid-template-rows: 1fr auto;
  grid-template-areas:
    'mapdisplay'
    'controls';
  overflow: scroll;
  background-repeat: no-repeat;
`

const Controls = styled.div`
  grid-area: controls;
`

function clamp(x: number, min: number, max: number): number {
  return Math.min(Math.max(x, min), max)
}

const locations = {
  Bangalore: { lon: 77.536404, lat: 13.170847 },
  'Green River': { lon: -110.139383, lat: 38.99218 },
  MDRS: { lon: -110.791683, lat: 38.406359 },
  D1: { lon: 19.919152, lat: 50.064588 },
}

export const OldMap: () => JSX.Element = () => {
  const [mapPath, setMapPath] = useState(ImagePaths.STATIC_COSTMAP)

  const mapContainerRef = useRef<HTMLDivElement>(null)

  const [markerLat, setMarkerLat] = React.useState('0')
  const [markerLon, setMarkerLon] = React.useState('0')
  const [markers, setMarkers] = React.useState<Array<Marker>>([])
  const [tags, setTags] = React.useState<Array<Marker>>([])
  const [followRover, setFollowRover] = React.useState(false)
  const [tagsEnabled, setTagsEnabled] = React.useState(true)
  const [ircMode, setIrcMode] = React.useState(false)
  const [magicNumbers] = React.useState<Array<number>>([0.0000089, 0.0000139])
  const [originPoint] = React.useState<Array<number>>([50.064775, 19.9195])
  // const [magicNumbers, setMagicNumbers] = React.useState<Array<number>>([0.0000089, 0.0000139])
  // const [originPoint, setOriginPoint] = React.useState<Array<number>>([50.06484, 19.919427])

  const roverLat = useAppSelector((state) => state.autonomy.gps.position.latitude)
  const roverLon = useAppSelector((state) => state.autonomy.gps.position.longitude)

  const roverMapLat = useAppSelector((state) => state.autonomy.map.position.y)
  const roverMapLon = useAppSelector((state) => state.autonomy.map.position.x)

  const goal = useAppSelector((state) => ({
    lat: state.autonomy.usp?.x || 0,
    lon: state.autonomy.usp?.y || 0,
  }))

  // const defaultRoverPosition = locations['Green River']
  const defaultRoverPosition = locations['D1']

  const angle = useAppSelector((state) => state.autonomy.imu.gamma)
  const [roverPosition, setRoverPosition] = React.useState(defaultRoverPosition)

  const [map, setMap] = React.useState<MapType>()
  const [roverMarker, setRoverMarker] = React.useState<Marker>()
  const [goalMarker, setGoalMarker] = React.useState<Marker>()

  const resetMap: () => void = () => {
    map?.jumpTo({
      center: roverPosition,
      zoom: 18,
    })
    // map?.resize()
  }

  const jumpToGoal: () => void = () => {
    map?.jumpTo({
      center: goal,
      zoom: 18,
    })
  }

  const addMarker: () => void = () => {
    if (map) {
      const marker = new maplibregl.Marker()
        .setLngLat({
          lat: parseFloat(markerLat),
          lon: parseFloat(markerLon),
        })
        .addTo(map)

      markers.push(marker)
    }
  }

  const clearMarkers: () => void = () => {
    for (const marker of markers) {
      marker.remove()
    }
    setMarkers([])
  }

  const showTags: () => void = () => {
    if (map) {
      const request = getRequest('/station/system/rover/autonomy/tags_positions') as Promise<
        AxiosResponse<Array<Array<number>>, unknown>
      >
      request.then((res: AxiosResponse<Array<Array<number>>, unknown>) => {
        res.data.forEach((tag) => {
          const el = document.createElement('div')
          el.className = 'tagMarker'
          el.innerText = tag[0].toString()

          const marker = new maplibregl.Marker(el)
            .setLngLat([magicNumbers[1] * tag[1] + originPoint[1], magicNumbers[0] * tag[2] + originPoint[0]])
            .addTo(map)
          tags.push(marker)
        })
      })
    }
  }

  const showWaypoints: () => void = () => {
    if (map) {
      const request = getRequest('/station/system/rover/autonomy/waypoints_positions') as Promise<
        AxiosResponse<Array<Array<number>>, unknown>
      >
      request.then((res: AxiosResponse<Array<Array<number>>, unknown>) => {
        res.data.forEach((tag) => {
          const el = document.createElement('div')
          el.className = 'tagMarker'
          el.innerText = tag[0].toString()

          const marker = new maplibregl.Marker(el)
            .setLngLat([magicNumbers[1] * tag[1] + originPoint[1], magicNumbers[0] * tag[2] + originPoint[0]])
            .addTo(map)
          tags.push(marker)
        })
      })
    }
  }

  // const mapClickHandler = (e) => {
  //     if (map) {
  //         console.log("A click event has occurred at " + e.lngLat);

  //             new maplibregl.Popup()
  //                 .setLngLat(e.lngLat)
  //                 .setHTML(
  //                     if (tagsEnabled) {
  //             `<h1>${((originPoint[1] - e.lngLat.lat) * magicNumbers[1]).toFixed(6)},
  // ${((originPoint[0] - e.lngLat.lng) * magicNumbers[0]).toFixed(6)}</h1>`
  //         } else{`<h1>${e.lngLat.lat.toFixed(6)}, ${e.lngLat.lng.toFixed(6)}</h1>`}
  //                 )
  //                 .addTo(map);
  //         } else {
  //             new maplibregl.Popup()
  //                 .setLngLat(e.lngLat)
  //                 .setHTML(
  //                     `<h1>${e.lngLat.lat.toFixed(6)}, ${e.lngLat.lng.toFixed(6)}</h1>`
  //                 )
  //                 .addTo(map);
  //         }
  //     }
  // }

  const clearTags: () => void = () => {
    for (const marker of tags) {
      marker.remove()
    }
    setTags([])
  }

  useEffect(() => {
    if (tagsEnabled) {
      if (angle) roverMarker?.setRotation(-((angle / 3.14) * 180))
      setRoverPosition({
        lon: magicNumbers[1] * (roverMapLon || defaultRoverPosition.lon) + originPoint[1],
        lat: magicNumbers[0] * (roverMapLat || defaultRoverPosition.lat) + originPoint[0],
      })
    } else {
      if (angle) roverMarker?.setRotation(-((angle / 3.14) * 180 - 90))
      setRoverPosition({
        lon: clamp(roverLon || defaultRoverPosition.lon, -90, 90),
        lat: clamp(roverLat || defaultRoverPosition.lat, -90, 90),
      })
    }
  }, [angle, roverLat, roverLon, roverMapLat, roverMapLon])

  useEffect(() => {
    roverMarker?.setLngLat(roverPosition)
    if (followRover) resetMap()
  }, [roverPosition])

  useEffect(() => {
    goalMarker?.setLngLat(goal)
  }, [goal])

  useEffect(() => {
    if (tagsEnabled) {
      showTags()
      showWaypoints()
    } else {
      clearTags()
    }
  }, [tagsEnabled, magicNumbers, originPoint, map])

  useEffect(() => {
    if (map) {
      map.on('drag', () => {
        setFollowRover(false)
      })

      const markerElement = document.createElement('div')
      markerElement.style.width = '24px'
      markerElement.style.height = '24px'
      markerElement.style.backgroundImage = 'url("http://localhost:8081/tiles/arrow-24.png")'

      const marker = new maplibregl.Marker(markerElement).setLngLat(roverPosition).addTo(map)
      setRoverMarker(marker)

      const gMarker = new maplibregl.Marker({ color: '#00FF00' }).setLngLat(goal).addTo(map)
      setGoalMarker(gMarker)

      if (tagsEnabled) {
        showTags()
        showWaypoints()
      }

      // const request = getRequest('/station/system/rover/autonomy/map_transform') as Promise<
      //   AxiosResponse<Array<Array<number>>, unknown>
      // >
      // request
      //   .then((res: AxiosResponse<Array<Array<number>>, unknown>) => {
      //     setOriginPoint(res.data[0])
      //     setMagicNumbers(res.data[1])
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      map.on('click', (e: any) => {
        if (e.originalEvent.explicitOriginalTarget.className.includes('tagMarker')) {
          return
        }
        console.log('A click event has occurred at ' + e.lngLat)
        console.log(originPoint[0] - e.lngLat.lat)
        new maplibregl.Popup()
          .setLngLat(e.lngLat)
          .setHTML(
            tagsEnabled
              ? `<h1> ${((e.lngLat.lng - originPoint[1]) / magicNumbers[1]).toFixed(6)},${(
                  (e.lngLat.lat - originPoint[0]) /
                  magicNumbers[0]
                ).toFixed(6)}</h1>`
              : `<h1>${e.lngLat.lat.toFixed(6)}, ${e.lngLat.lng.toFixed(6)}</h1>`,
          )
          .addTo(map)
      })
      // })
      // .catch(() => {
      //   console.log('ERROR while requesting map transform')
      // })
    }
  }, [map])

  useEffect(() => {
    setMap(new maplibregl.Map(mapDataD1(mapContainerRef)))

    setRoverPosition({
      lon: roverLon || defaultRoverPosition.lon,
      lat: roverLat || defaultRoverPosition.lat,
    })

    return () => {
      map?.remove()
    }
  }, [])

  const handleImageTypeChange: () => void = () => {
    if (mapPath === ImagePaths.GRAYSCALE) {
      setMapPath(ImagePaths.STATIC_COSTMAP)
    } else {
      setMapPath(ImagePaths.GRAYSCALE)
    }
  }

  useEffect(() => {
    console.log(mapPath)
    if (mapPath === ImagePaths.GRAYSCALE) {
      map?.setLayoutProperty('marsyard-layer-grayscale', 'visibility', 'none')
      map?.setLayoutProperty('marsyard-layer-costmap', 'visibility', 'visible')
    } else {
      map?.setLayoutProperty('marsyard-layer-grayscale', 'visibility', 'visible')
      map?.setLayoutProperty('marsyard-layer-costmap', 'visibility', 'none')
    }
  }, [mapPath])

  return (
    <Wrapper>
      <div ref={mapContainerRef} style={{ gridArea: 'mapdisplay' }}></div>
      <Controls>
        <button onClick={handleImageTypeChange} style={{ border: '2px solid red', padding: '5px' }}>
          Toggle image type
        </button>
        <button
          onClick={(): void => {
            resetMap()
            setFollowRover(true)
          }}
        >
          Reset to rover
        </button>{' '}
        Latitude:
        <input
          type='text'
          value={markerLat}
          onChange={(e: React.ChangeEvent<HTMLInputElement>): void => setMarkerLat(() => e.target.value)}
        />{' '}
        Longitude:
        <input
          type='text'
          value={markerLon}
          onChange={(e: React.ChangeEvent<HTMLInputElement>): void => setMarkerLon(() => e.target.value)}
        />
        <button onClick={addMarker}>add marker</button>
        <button onClick={clearMarkers}>CLEAR MARKERS</button>
        Follow the rover:{' '}
        <input type='checkbox' min={0} max={1} checked={followRover} onChange={(): void => setFollowRover((s) => !s)} />
        <button
          onClick={(): void => {
            jumpToGoal()
          }}
        >
          Jump to goal
        </button>{' '}
        Map local mode:{' '}
        <input type='checkbox' min={0} max={1} checked={tagsEnabled} onChange={(): void => setTagsEnabled((s) => !s)} />
        IRC mode:{' '}
        <input type='checkbox' min={0} max={1} checked={ircMode} onChange={(): void => setIrcMode((s) => !s)} />
      </Controls>
    </Wrapper>
  )
}
