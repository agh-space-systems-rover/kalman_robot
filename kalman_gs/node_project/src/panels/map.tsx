import styles from './map.module.css';

import kalmanMarker from '!!url-loader!../media/kalman-marker.svg';
import waypointMarker from '!!url-loader!../media/waypoint-marker.svg';
import { gpsCoords, hasGpsCoords } from '../common/gps';
import { imuRotation } from '../common/imu';
import '../common/leaflet-rotated-marker-plugin';
import { mapMarker, setMapMarkerLatLon } from '../common/map-marker';
import { Quaternion, Vector3, quatConj, quatTimesVec } from '../common/mini-math-lib';
import { ros } from '../common/ros';
import { GeoPoint, GeoPath } from '../common/ros-interfaces';
import { waypoints } from '../common/waypoints';
import erc2025Overlay from '../media/erc2025-overlay.png';
import { faCloudSun, faCopy, faGlobe, faRobot } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import Leaflet from 'leaflet';
import icon from 'leaflet/dist/images/marker-icon.png';
import iconShadow from 'leaflet/dist/images/marker-shadow.png';
import 'leaflet/dist/leaflet.css';
import { Component, createRef, useState, useEffect, useCallback } from 'react';
import { ImageOverlay, MapContainer, Marker, ScaleControl, TileLayer, Tooltip, Polyline } from 'react-leaflet';
import { Topic } from 'roslib';

import Button from '../components/button';

const GO_TO_LOCATION_ZOOM = 19;
const DEFAULT_LAT = 51.477928;
const DEFAULT_LONG = -0.001545;
const DEFAULT_ZOOM = 18;
const PROPS_UPDATE_INTERVAL = 100;

type SolarConjunctionPoint = {
  latitude: number;
  longitude: number;
};

const SOLAR_CONJUNCTION_POINT_STORAGE_KEY = 'solar-conjunction-point';

let savedSolarConjunctionPoint: SolarConjunctionPoint | null = null;
const savedSolarConjunctionPointJson = localStorage.getItem(SOLAR_CONJUNCTION_POINT_STORAGE_KEY);
if (savedSolarConjunctionPointJson) {
  try {
    savedSolarConjunctionPoint = JSON.parse(savedSolarConjunctionPointJson);
  } catch {
    localStorage.removeItem(SOLAR_CONJUNCTION_POINT_STORAGE_KEY);
  }
}

Leaflet.Marker.prototype.options.icon = Leaflet.icon({
  iconUrl: icon,
  shadowUrl: iconShadow,
  iconAnchor: [12, 41]
});

let geoPathArray: GeoPoint[] = [];
window.addEventListener('ros-connect', () => {
  const geoPathTopic = new Topic({
    ros: ros,
    name: '/plan/gps',
    messageType: 'kalman_interfaces/GeoPath'
  });

  geoPathTopic.subscribe((msg: GeoPath) => {
    geoPathArray = msg.points;
    window.dispatchEvent(new CustomEvent('map-geo-points-update'));
  });
});

type Props = {
  props: {
    viewLat: number;
    viewLong: number;
    viewZoom: number;
  };
};

type State = {
  solarConjunctionPoint: SolarConjunctionPoint | null;
};

function headingFromRoverRot(baseToMap: Quaternion): number {
  const northMap: Vector3 = {
    x: 0,
    y: 1,
    z: 0
  }; // north in map frame
  const northBase = quatTimesVec(quatConj(baseToMap), northMap);
  const angleFromHeadingToNorth = Math.atan2(northBase.y, northBase.x);
  // angle to north vector will increase when turning right
  // heading shall behave the same way
  const heading = (angleFromHeadingToNorth * 180) / Math.PI;
  return heading;
}

/**
 * displaying gps coordinates
 */
function GpsCoordinatesDisplay({ solarConjunctionPoint }: State) {
  const [markerCoords, setMarkerCoords] = useState({
    lat: mapMarker.latitude,
    long: mapMarker.longitude
  });
  const [roverCoords, setRoverCoords] = useState({
    lat: gpsCoords.latitude,
    long: gpsCoords.longitude
  });

  const onMarkerUpdated = useCallback(() => {
    setMarkerCoords({
      lat: mapMarker.latitude,
      long: mapMarker.longitude
    });
  }, []);

  const onGpsUpdated = useCallback(() => {
    setRoverCoords({
      lat: gpsCoords.latitude,
      long: gpsCoords.longitude
    });
  }, []);

  useEffect(() => {
    window.addEventListener('map-marker-move', onMarkerUpdated);
    window.addEventListener('gps-update', onGpsUpdated);
    return () => {
      window.removeEventListener('map-marker-move', onMarkerUpdated);
      window.removeEventListener('gps-update', onGpsUpdated);
    };
  }, [onMarkerUpdated, onGpsUpdated]);

  return (
    <div className={styles['coordinates-display-controls']}>
      {roverCoords.lat !== undefined && roverCoords.long !== undefined && (
        <div className={`${styles['gps-display-control']} ${styles['rover-coordinates-control']}`}>
          <FontAwesomeIcon icon={faRobot} />
          <div className={styles['gps-coords-text']}>
            {`${roverCoords.lat.toFixed(6)}, ${roverCoords.long.toFixed(6)}`}
          </div>
          <Button
            className={styles['rover-copy-button']}
            tooltip='Copy rover coordinates to clipboard.'
            onClick={() => {
              navigator.clipboard.writeText(`${roverCoords.lat.toFixed(8)}, ${roverCoords.long.toFixed(8)}`);
            }}
          >
            <FontAwesomeIcon icon={faCopy} />
          </Button>
        </div>
      )}
      {solarConjunctionPoint && (
        <div className={`${styles['gps-display-control']} ${styles['solar-coordinates-control']}`}>
          <FontAwesomeIcon icon={faCloudSun} />
          <div className={styles['gps-coords-text']}>
            {`${solarConjunctionPoint.latitude.toFixed(6)}, ${solarConjunctionPoint.longitude.toFixed(6)}`}
          </div>
          <Button
            className={styles['solar-copy-button']}
            tooltip='Copy Solar Conjunction Point coordinates to clipboard.'
            onClick={() => {
              navigator.clipboard.writeText(
                `${solarConjunctionPoint.latitude.toFixed(8)}, ${solarConjunctionPoint.longitude.toFixed(8)}`
              );
            }}
          >
            <FontAwesomeIcon icon={faCopy} />
          </Button>
        </div>
      )}
      <div className={styles['gps-display-control']}>
        <FontAwesomeIcon icon={faGlobe} />
        <div className={styles['gps-coords-text']}>
          {`${markerCoords.lat.toFixed(6)}, ${markerCoords.long.toFixed(6)}`}
        </div>
        <Button
          tooltip='Copy marker coordinates to clipboard.'
          onClick={() => {
            navigator.clipboard.writeText(`${mapMarker.latitude.toFixed(8)}, ${mapMarker.longitude.toFixed(8)}`);
          }}
        >
          <FontAwesomeIcon icon={faCopy} />
        </Button>
      </div>
    </div>
  );
}
export default class Map extends Component<Props, State> {
  state: State = {
    solarConjunctionPoint: savedSolarConjunctionPoint
  };

  private mapRef = createRef<Leaflet.Map>();
  private mapMarkerRef = createRef<Leaflet.Marker>();
  private kalmanMarkerRef = createRef<Leaflet.Marker>();
  private polylineRef = createRef<Leaflet.Polyline>();
  private propsUpdateTimer: NodeJS.Timeout | null = null;

  private updateProps = () => {
    if (this.mapRef.current) {
      const { props } = this.props;

      props.viewLat = this.mapRef.current.getCenter().lat;
      props.viewLong = this.mapRef.current.getCenter().lng;
      props.viewZoom = this.mapRef.current.getZoom();
    }
  };

  private onResized = () => {
    this.mapRef.current?.invalidateSize();
  };

  private onMapMarkerMoved = () => {
    this.mapMarkerRef.current?.setLatLng([mapMarker.latitude, mapMarker.longitude]);
  };

  private onImuUpdated = () => {
    // Here we use our custom injected method to set the rotation angle of the marker.
    // Cast to any because the type definitions are not up to date.
    (this.kalmanMarkerRef.current as any)?.setRotationAngle(headingFromRoverRot(imuRotation)); // 149.5001
  };

  private onGpsUpdated = () => {
    this.kalmanMarkerRef.current?.setLatLng([gpsCoords.latitude, gpsCoords.longitude]);
  };

  private onWaypointsUpdated = () => {
    this.forceUpdate();
  };

  private onMapgeoPathUpdated = () => {
    this.polylineRef.current?.setLatLngs(geoPathArray.map((point) => [point.latitude, point.longitude]));
  };

  componentDidMount() {
    if (window) {
      window.addEventListener('resize', this.onResized);
      window.addEventListener('any-panel-resize', this.onResized);
      window.addEventListener('map-marker-move', this.onMapMarkerMoved);
      window.addEventListener('imu-update', this.onImuUpdated);
      window.addEventListener('gps-update', this.onGpsUpdated);
      window.addEventListener('waypoints-update', this.onWaypointsUpdated);
      window.addEventListener('map-geo-points-update', this.onMapgeoPathUpdated);
    }

    this.propsUpdateTimer = setInterval(() => {
      this.updateProps();
    }, PROPS_UPDATE_INTERVAL);
  }

  componentWillUnmount() {
    if (this.propsUpdateTimer) {
      clearInterval(this.propsUpdateTimer);
    }

    if (window) {
      window.removeEventListener('gps-update', this.onGpsUpdated);
      window.removeEventListener('imu-update', this.onImuUpdated);
      window.removeEventListener('map-marker-move', this.onMapMarkerMoved);
      window.removeEventListener('resize', this.onResized);
      window.removeEventListener('any-panel-resize', this.onResized);
      window.removeEventListener('waypoints-update', this.onWaypointsUpdated);
      window.removeEventListener('map-geo-points-update', this.onMapgeoPathUpdated);
    }
  }

  goToLocation = (lat: number, long: number, keepZoom = false) => {
    if (keepZoom) {
      this.mapRef.current?.setView([lat, long]);
    } else {
      this.mapRef.current?.setView([lat, long], GO_TO_LOCATION_ZOOM);
    }
  };

  getCurrentLocation = () => {
    return {
      latitude: this.mapRef.current.getCenter().lat,
      longitude: this.mapRef.current.getCenter().lng
    };
  };

  getSolarConjunctionPoint = () => {
    return this.state.solarConjunctionPoint;
  };

  setSolarConjunctionPoint = (latitude: number, longitude: number) => {
    const solarConjunctionPoint = { latitude, longitude };
    localStorage.setItem(SOLAR_CONJUNCTION_POINT_STORAGE_KEY, JSON.stringify(solarConjunctionPoint));
    this.setState({ solarConjunctionPoint });
  };

  removeSolarConjunctionPoint = () => {
    localStorage.removeItem(SOLAR_CONJUNCTION_POINT_STORAGE_KEY);
    this.setState({ solarConjunctionPoint: null });
  };

  render() {
    const { props } = this.props;
    const { solarConjunctionPoint } = this.state;
    if (props.viewLat === undefined) {
      props.viewLat = gpsCoords.latitude || DEFAULT_LAT;
    }
    if (props.viewLong === undefined) {
      props.viewLong = gpsCoords.longitude || DEFAULT_LONG;
    }
    if (props.viewZoom === undefined) {
      props.viewZoom = DEFAULT_ZOOM;
    }

    return (
      <div className={styles['map']}>
        <MapContainer
          className={styles['map-container']}
          ref={this.mapRef}
          center={[props.viewLat, props.viewLong]}
          zoom={props.viewZoom}
          scrollWheelZoom={true}
          attributionControl={false}
          zoomControl={false}
          maxBounds={[
            [90, -180],
            [-90, 180]
          ]}
          maxBoundsViscosity={1.0}
          keyboard={false}
        >
          <TileLayer
            // url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
            // subdomains={['mt0', 'mt1', 'mt2', 'mt3']}
            // Instead, use MapProxy which will cache the tiles for faster access.
            // This should also prevent Google from throttling the requests.
            // In case of lost internet connection, the tiles will still be available.
            url='http://localhost:8065/wmts/gm_layer/gm_grid/{z}/{x}/{y}.png'
            maxNativeZoom={19}
            maxZoom={23}
            minZoom={3}
          />
          {/*<ImageOverlay
            url={erc2024Overlay}
            bounds={[
              // Order of points does not matter as long as they are diagonally opposite corners of the image:
              [50.0663741908217, 19.9130491956501],
              [50.0659224467215, 19.9137533400464]
            ]}
          />*/}
          <ImageOverlay
            url={erc2025Overlay}
            bounds={[
              // Order of points does not matter as long as they are diagonally opposite corners of the image:
              [50.065971191749, 19.9130887981378],
              [50.066363153534, 19.9137014499564]
            ]}
          />
          {hasGpsCoords() && (
            <Marker
              ref={this.kalmanMarkerRef}
              position={[gpsCoords.latitude, gpsCoords.longitude]}
              interactive={false}
              icon={Leaflet.icon({
                className: styles['kalman-marker'],
                iconUrl: kalmanMarker,
                iconAnchor: [25, 25],
                iconSize: [50, 50]
              })}
            />
          )}
          {solarConjunctionPoint && (
            <Marker
              position={[solarConjunctionPoint.latitude, solarConjunctionPoint.longitude]}
              interactive={false}
              icon={Leaflet.icon({
                className: `${styles['waypoint-marker']} yellow`,
                iconUrl: waypointMarker,
                iconAnchor: [10.2, 45],
                iconSize: [45, 45]
              })}
            >
              <Tooltip
                direction='right'
                offset={[0, 0]}
                opacity={1}
                permanent
                className={styles['waypoint-marker-tooltip']}
              >
                Solar Conjunction Point
              </Tooltip>
            </Marker>
          )}
          {waypoints.map((marker, i) => (
            <Marker
              key={i}
              position={[marker.lat, marker.lon]}
              interactive={false}
              icon={Leaflet.icon({
                className: styles['waypoint-marker'] + ' ' + marker.color,
                iconUrl: waypointMarker,
                iconAnchor: [10.2, 45],
                iconSize: [45, 45]
              })}
            >
              <Tooltip
                direction='right'
                offset={[0, 0]}
                opacity={1}
                permanent
                className={styles['waypoint-marker-tooltip']}
              >
                {marker.name}
              </Tooltip>
            </Marker>
          ))}
          <Marker
            ref={this.mapMarkerRef}
            position={[mapMarker.latitude, mapMarker.longitude]}
            draggable
            autoPan
            eventHandlers={{
              drag: (event) => {
                const marker = event.target;
                const position = marker.getLatLng();
                setMapMarkerLatLon(position.lat, position.lng);
              }
            }}
            riseOnHover
          />
          <ScaleControl imperial={false} maxWidth={200} />

          <Polyline
            ref={this.polylineRef}
            positions={geoPathArray.map((point) => [point.latitude, point.longitude])}
            color={'darkblue'}
            weight={5}
            opacity={0.8}
          />
        </MapContainer>
        <GpsCoordinatesDisplay solarConjunctionPoint={solarConjunctionPoint} />
      </div>
    );
  }
}
