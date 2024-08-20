import styles from './map.header.module.css';

import { gpsCoords } from '../common/gps';
import { mapMarker, setMapMarkerLatLon } from '../common/map-marker';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { NavSatFix, SpoofGpsRequest } from '../common/ros-interfaces';
import Map from './map';
import {
  faCopy,
  faJetFighterUp,
  faLocationDot,
  faPaperPlane,
  faRobot,
  faSearch,
  faThumbTack,
  faUpDownLeftRight
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import {
  MutableRefObject,
  createRef,
  useCallback,
  useEffect,
  useRef,
  useState
} from 'react';
import { Service, Topic } from 'roslib';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

let spoofGpsSrv: Service<SpoofGpsRequest, {}> = null;
window.addEventListener('ros-connect', () => {
  spoofGpsSrv = new Service<SpoofGpsRequest, {}>({
    ros: ros,
    name: '/spoof_gps',
    serviceType: 'kalman_robot/SupervisorGpsSpoof'
  });
});

type Props = {
  panelRef: MutableRefObject<Map>;
};

async function parseLatLong(
  query: string
): Promise<[number, number] | undefined> {
  // Attempt to parse the query as a latitude and longitude pair.
  // If a trimmed query begins with a number and ends with another number with punctuation in between, it is likely a latitude and longitude pair.
  // 0. Check if the query does not contain alphabetic characters.
  if (!/[a-zA-Z]/.test(query)) {
    // 1. Trim the query to remove leading and trailing whitespace.
    const trimmedQuery = query.trim();
    // 2. Replace non-numerics with spaces to then check for numbers.
    const numericQuery = trimmedQuery
      .replace(/[^0-9. -]/g, ' ')
      .replaceAll(/\s+/g, ' ');
    // 3. Split the query into numbers.
    const numbers = numericQuery
      .split(' ')
      .map(Number)
      .filter((n) => !isNaN(n));
    // 3. If there are exactly two numbers, return them as the latitude and longitude.
    if (numbers.length === 2) {
      return [numbers[0], numbers[1]];
    }
  }

  // Geocode the query using a free and open source Photon API.
  const res = await fetch(
    `https://photon.komoot.io/api/?q=${encodeURIComponent(query)}`
  );

  // If the request failed, log the response and return undefined.
  if (!res.ok) {
    console.error('Failed to send a geocode request:\n', res);
    return;
  }

  // Parse the response as JSON and check if a place was found.
  const data = await res.json();
  if (!data.features.length) {
    return;
  }

  // Return the latitude and longitude of the first result.
  return [
    data.features[0].geometry.coordinates[1] as number,
    data.features[0].geometry.coordinates[0] as number
  ];
}

export default function MapHeader({ panelRef }: Props) {
  const [rerenderCount, setRerenderCount] = useState(0);
  const inputRef = useRef<Input>();

  const rerender = useCallback(() => {
    setRerenderCount(rerenderCount + 1);
  }, [rerenderCount]);

  const goToGeocode = useCallback(async () => {
    const query = inputRef.current?.getValue();

    try {
      const latLong = await parseLatLong(query);
      if (latLong) {
        panelRef.current?.goToLocation(...latLong);
      } else {
        alert('No results found.');
      }
    } catch (err) {
      alertsRef.current?.pushAlert(
        'Failed to reach the Geocoding service. Please check your internet connection.'
      );
    }
  }, [panelRef]);

  useEffect(() => {
    window.addEventListener('gps-update', rerender);
    return () => {
      window.removeEventListener('gps-update', rerender);
    };
  }, [rerender]);

  return (
    <div className={styles['map-header']}>
      <div className={styles['section'] + ' ' + styles['search-section']}>
        <Input
          ref={inputRef}
          placeholder='Search for a location...'
          onSubmit={goToGeocode}
          onChange={rerender}
        />
        <Button
          onClick={goToGeocode}
          disabled={!(inputRef.current?.isEmpty() === false)}
        >
          <FontAwesomeIcon icon={faSearch} />
        </Button>
      </div>
      <div className={styles['section']}>
        <Label className={styles['rover-label']}>
          <FontAwesomeIcon icon={faRobot} />
        </Label>
        <Button
          tooltip='Fly to the rover.'
          onClick={() => {
            panelRef.current?.goToLocation(
              gpsCoords.latitude,
              gpsCoords.longitude,
              true
            );
          }}
          disabled={!gpsCoords.latitude || !gpsCoords.longitude}
        >
          <FontAwesomeIcon icon={faPaperPlane} />
        </Button>
        <Button
          tooltip='Set rover location on the marker.\nThis is useful when navigating without GPS.'
          onClick={() => {
            if (!spoofGpsSrv) {
              alertsRef.current?.pushAlert(
                'Failed to set rover location. ROS connection is not established.'
              );
              return;
            }

            const req: SpoofGpsRequest = {
              location: {
                latitude: mapMarker.latitude,
                longitude: mapMarker.longitude
              }
            };
            spoofGpsSrv.callService(req, undefined, (error: string) => {
              alertsRef.current?.pushAlert(
                'Failed to set rover location. ' + error
              );
            });
          }}
        >
          <FontAwesomeIcon icon={faUpDownLeftRight} />
        </Button>
        <Button
          tooltip='Copy rover coordinates to clipboard.'
          onClick={() => {
            navigator.clipboard.writeText(
              `${gpsCoords.latitude.toFixed(8)}, ${gpsCoords.longitude.toFixed(8)}`
            );
          }}
          disabled={!gpsCoords.latitude || !gpsCoords.longitude}
        >
          <FontAwesomeIcon icon={faCopy} />
        </Button>
      </div>
      <div className={styles['section']}>
        <Label className={styles['marker-label']}>
          <FontAwesomeIcon icon={faLocationDot} />
        </Label>
        <Button
          tooltip='Fly to the marker.'
          onClick={() => {
            panelRef.current?.goToLocation(
              mapMarker.latitude,
              mapMarker.longitude,
              true
            );
          }}
        >
          <FontAwesomeIcon icon={faPaperPlane} />
        </Button>
        <Button
          tooltip='Set marker location in the center.'
          onClick={() => {
            const location = panelRef.current?.getCurrentLocation();
            if (location) {
              setMapMarkerLatLon(location.latitude, location.longitude);
            }
          }}
        >
          <FontAwesomeIcon icon={faThumbTack} />
        </Button>
        <Button
          tooltip='Copy marker coordinates to clipboard.'
          onClick={() => {
            navigator.clipboard.writeText(
              `${mapMarker.latitude.toFixed(8)}, ${mapMarker.longitude.toFixed(8)}`
            );
          }}
        >
          <FontAwesomeIcon icon={faCopy} />
        </Button>
      </div>
    </div>
  );
}
