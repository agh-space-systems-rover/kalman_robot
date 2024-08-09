import styles from './map.module.css';

import kalmanMarker from '!!url-loader!../media/kalman-marker.svg';
import { gpsFix } from '../common/gps';
import { imuRotation } from '../common/imu';
import '../common/leaflet-rotated-marker-plugin';
import { mapMarker } from '../common/map-marker';
import { Quaternion, Vector3, quatTimesVec } from '../common/mini-math-lib';
import Leaflet from 'leaflet';
import icon from 'leaflet/dist/images/marker-icon.png';
import iconShadow from 'leaflet/dist/images/marker-shadow.png';
import 'leaflet/dist/leaflet.css';
import { Component, createRef } from 'react';
import { MapContainer, Marker, TileLayer } from 'react-leaflet';

const GO_TO_LOCATION_ZOOM = 17;
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

export default class Map extends Component<Props> {
  private mapRef = createRef<any>();
  private mapMarkerRef = createRef<any>();
  private kalmanMarkerRef = createRef<any>();
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
    this.mapMarkerRef.current?.setLatLng([
      mapMarker.latitude,
      mapMarker.longitude
    ]);
  };

  private onImuUpdated = () => {
    this.kalmanMarkerRef.current?.setRotationAngle(
      -((rosYawFromQuat(imuRotation) - ROS_IMU_LINK_YAW) * 180) / Math.PI
    );
  };

  private onGpsUpdated = () => {
    this.kalmanMarkerRef.current?.setLatLng([
      gpsFix.latitude,
      gpsFix.longitude
    ]);
  };

  componentDidMount() {
    if (window) {
      window.addEventListener('resize', this.onResized);
      window.addEventListener('any-panel-resize', this.onResized);
      window.addEventListener('map-marker-move', this.onMapMarkerMoved);
      window.addEventListener('imu-update', this.onImuUpdated);
      window.addEventListener('gps-update', this.onGpsUpdated);
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
    }
  }

  goToLocation = (lat: number, long: number) => {
    this.mapRef.current?.setView([lat, long], GO_TO_LOCATION_ZOOM);
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
      props.viewLat = gpsFix.latitude || DEFAULT_LAT;
    }
    if (props.viewLong === undefined) {
      props.viewLong = gpsFix.longitude || DEFAULT_LONG;
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
          <Marker
            ref={this.mapMarkerRef}
            position={[mapMarker.latitude, mapMarker.longitude]}
            draggable={true}
            autoPan={true}
            eventHandlers={{
              drag: (event) => {
                const marker = event.target;
                const position = marker.getLatLng();
                mapMarker.latitude = position.lat;
                mapMarker.longitude = position.lng;
                window.dispatchEvent(new CustomEvent('map-marker-move'));
              }
            }}
          />
          <Marker
            ref={this.kalmanMarkerRef}
            position={[gpsFix.latitude || 1000000, gpsFix.longitude || 1000000]}
            icon={Leaflet.icon({
              className: styles['kalman-marker'],
              iconUrl: kalmanMarker,
              iconAnchor: [25, 25],
              iconSize: [50, 50]
            })}
          />
        </MapContainer>
      </div>
    );
  }
}
