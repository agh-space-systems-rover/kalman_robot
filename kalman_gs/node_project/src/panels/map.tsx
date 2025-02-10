import styles from './map.module.css';

import kalmanMarker from '!!url-loader!../media/kalman-marker.svg';
import waypointMarker from '!!url-loader!../media/waypoint-marker.svg';
import { gpsCoords } from '../common/gps';
import { imuRotation } from '../common/imu';
import '../common/leaflet-rotated-marker-plugin';
import { mapMarker, setMapMarkerLatLon } from '../common/map-marker';
import { Quaternion, Vector3, quatTimesVec } from '../common/mini-math-lib';
import { waypoints } from '../common/waypoints';
import erc2024Overlay from '../media/erc2024-overlay.png';
import Leaflet from 'leaflet';
import icon from 'leaflet/dist/images/marker-icon.png';
import iconShadow from 'leaflet/dist/images/marker-shadow.png';
import 'leaflet/dist/leaflet.css';
import { Component, createRef } from 'react';
import { ImageOverlay, MapContainer, Marker, ScaleControl, TileLayer, Tooltip } from 'react-leaflet';

const GO_TO_LOCATION_ZOOM = 19;
const DEFAULT_LAT = 51.477928;
const DEFAULT_LONG = -0.001545;
const DEFAULT_ZOOM = 18;
const PROPS_UPDATE_INTERVAL = 100;
const ROS_IMU_LINK_YAW = 1.57;

Leaflet.Marker.prototype.options.icon = Leaflet.icon({
  iconUrl: icon,
  shadowUrl: iconShadow,
  iconAnchor: [12, 41]
});

type Props = {
  props: {
    viewLat: number;
    viewLong: number;
    viewZoom: number;
  };
};

function rosYawFromQuat(q: Quaternion): number {
  const heading: Vector3 = {
    x: 0,
    y: 1,
    z: 0
  }; // north
  const rotatedHeading = quatTimesVec(q, heading);
  return Math.atan2(rotatedHeading.y, rotatedHeading.x);
}

const TEMP_POINTS = [
  { lat: 50.061405, lon: 19.930067 },
  { lat: 50.068225, lon: 19.939762 },
  { lat: 50.069588, lon: 19.933861 },
  { lat: 50.052832, lon: 19.937166 },
  { lat: 50.065096, lon: 19.941311 },
  { lat: 50.05236, lon: 19.940694 },
  { lat: 50.059168, lon: 19.93122 },
  { lat: 50.060242, lon: 19.947563 },
  { lat: 50.064876, lon: 19.946149 },
  { lat: 50.056217, lon: 19.92966 }
];

export default class Map extends Component<Props> {
  private mapRef = createRef<Leaflet.Map>();
  private mapMarkerRef = createRef<Leaflet.Marker>();
  private kalmanMarkerRef = createRef<Leaflet.Marker>();
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
    (this.kalmanMarkerRef.current as any)?.setRotationAngle(
      -((rosYawFromQuat(imuRotation) - ROS_IMU_LINK_YAW) * 180) / Math.PI
    );
  };

  private onGpsUpdated = () => {
    this.kalmanMarkerRef.current?.setLatLng([gpsCoords.latitude, gpsCoords.longitude]);
  };

  private onWaypointsUpdated = () => {
    this.forceUpdate();
  };

  componentDidMount() {
    if (window) {
      window.addEventListener('resize', this.onResized);
      window.addEventListener('any-panel-resize', this.onResized);
      window.addEventListener('map-marker-move', this.onMapMarkerMoved);
      window.addEventListener('imu-update', this.onImuUpdated);
      window.addEventListener('gps-update', this.onGpsUpdated);
      window.addEventListener('waypoints-update', this.onWaypointsUpdated);
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

  render() {
    const { props } = this.props;
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
          <ImageOverlay
            url={erc2024Overlay}
            bounds={[
              // Order of points does not matter as long as they are diagonally opposite corners of the image:
              [50.0663741908217, 19.9130491956501],
              [50.0659224467215, 19.9137533400464]
            ]}
          />
          <Marker
            ref={this.kalmanMarkerRef}
            position={[gpsCoords.latitude || 1000000, gpsCoords.longitude || 1000000]}
            interactive={false}
            icon={Leaflet.icon({
              className: styles['kalman-marker'],
              iconUrl: kalmanMarker,
              iconAnchor: [25, 25],
              iconSize: [50, 50]
            })}
          />
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
        </MapContainer>
      </div>
    );
  }
}
