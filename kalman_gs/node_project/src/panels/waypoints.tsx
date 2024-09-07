import styles from './waypoints.module.css';

import { mapMarker, setMapMarkerLatLon } from '../common/map-marker';
import { alertsRef } from '../common/refs';
import {
  addWaypoint,
  exportWaypointsAsText,
  importWaypointsFromText,
  removeAllWaypoints,
  removeWaypoint,
  waypointColors,
  waypoints
} from '../common/waypoints';
import {
  faArrowsLeftRight,
  faArrowsUpDown,
  faBan,
  faDownload,
  faFlag,
  faList,
  faLocationDot,
  faMapLocationDot,
  faPalette,
  faTrash,
  faUpload
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

export default function Waypoints() {
  const [rerenderCount, setRerenderCount] = useState(0);
  const [newWaypointColorI, setNewWaypointColorI] = useState(0);
  const latRef = useRef<Input>();
  const lonRef = useRef<Input>();
  const nameRef = useRef<Input>();
  const waypointCanBeCreated =
    latRef.current?.getValue() &&
    lonRef.current?.getValue() &&
    nameRef.current?.getValue();

  useEffect(() => {
    window.addEventListener('waypoints-update', () => {
      setRerenderCount(rerenderCount + 1);
    });
  }, [rerenderCount]);

  return (
    <div className={styles['waypoints']}>
      <div className={styles['scrollable']}>
        <div className={styles['row']}>
          <Label className={styles['latlon-label']}>
            <FontAwesomeIcon icon={faArrowsUpDown} />
          </Label>
          <Input
            type='float'
            placeholder='Latitude'
            ref={latRef}
            onChange={() => setRerenderCount(rerenderCount + 1)}
          />
          <Label className={styles['latlon-label']}>
            <FontAwesomeIcon icon={faArrowsLeftRight} />
          </Label>
          <Input
            type='float'
            placeholder='Longitude'
            ref={lonRef}
            onChange={() => setRerenderCount(rerenderCount + 1)}
          />
          <Button
            tooltip='Set waypoint location from map marker.'
            onClick={() => {
              const latTrunc = parseFloat(mapMarker.latitude.toFixed(8));
              const longTrunc = parseFloat(mapMarker.longitude.toFixed(8));
              latRef.current.setValue(latTrunc);
              lonRef.current.setValue(longTrunc);
              setRerenderCount(rerenderCount + 1);
            }}
          >
            <FontAwesomeIcon icon={faLocationDot} />
          </Button>
        </div>
        <div className={styles['row']}>
          <Button
            className={
              styles['colored-button'] + ' ' + waypointColors[newWaypointColorI]
            }
            tooltip='Set waypoint color.'
            onClick={() => {
              setNewWaypointColorI(
                (newWaypointColorI + 1) % waypointColors.length
              );
            }}
          >
            <FontAwesomeIcon icon={faPalette} />
          </Button>
          <Input
            className={styles['flex-grow']}
            placeholder='Choose waypoint name...'
            ref={nameRef}
            onChange={() => setRerenderCount(rerenderCount + 1)}
          />
          <Button
            tooltip='Create a waypoint.'
            disabled={!waypointCanBeCreated}
            onClick={() => {
              addWaypoint({
                lat: parseFloat(latRef.current.getValue()),
                lon: parseFloat(lonRef.current.getValue()),
                name: nameRef.current.getValue(),
                color: waypointColors[newWaypointColorI]
              });
              setRerenderCount(rerenderCount + 1);
            }}
          >
            <FontAwesomeIcon icon={faFlag} />
          </Button>
        </div>
        <div className={styles['row']}>
          <div className={styles['waypoint-list']}>
            <div className={styles['row']}>
              <Label
                className={
                  styles['waypoint-list-header'] + ' ' + styles['flex-grow']
                }
              >
                <FontAwesomeIcon icon={waypoints.length > 0 ? faList : faBan} />
                &nbsp; {waypoints.length > 0 ? '' : 'No'} Active Waypoints
              </Label>
            </div>
            {waypoints.map((waypoint, index) => (
              <div key={index} className={styles['row']}>
                <Label
                  className={styles['colored-label'] + ' ' + waypoint.color}
                >
                  &nbsp;
                </Label>
                <Label
                  className={
                    styles['flex-grow'] + ' ' + styles['waypoint-name-label']
                  }
                >
                  {waypoint.name}
                </Label>
                <Button
                  tooltip='Move marker to the waypoint.'
                  onClick={() => {
                    setMapMarkerLatLon(waypoint.lat, waypoint.lon);
                  }}
                >
                  <FontAwesomeIcon icon={faMapLocationDot} />
                </Button>
                <Button
                  tooltip='Remove waypoint'
                  onClick={() => {
                    // Ask for confirmation before removing the waypoint.
                    if (
                      window.confirm(
                        `Are you sure you want to remove "${waypoint.name}"?`
                      )
                    ) {
                      removeWaypoint(waypoint);
                      setRerenderCount(rerenderCount + 1);
                    }
                  }}
                >
                  <FontAwesomeIcon icon={faTrash} />
                </Button>
              </div>
            ))}
          </div>
        </div>
        <div className={styles['row']}>
          <Button
            className={styles['flex-grow']}
            onClick={() => {
              // Display a confirmation dialog.
              if (
                window.confirm('Are you sure you want to remove all waypoints?')
              ) {
                removeAllWaypoints();
              }
            }}
            disabled={waypoints.length === 0}
          >
            Remove All Waypoints
          </Button>
        </div>
        <div className={styles['row']}>
          <Button
            className={styles['flex-grow']}
            tooltip='Import waypoints from text.\nImported waypoints will be appended to the list.'
            onClick={() => {
              const text = prompt('Enter waypoints as text:');
              if (text) {
                const numWaypoints = waypoints.length;
                importWaypointsFromText(
                  text,
                  waypointColors[newWaypointColorI]
                );
                alertsRef.current?.pushAlert(
                  `Imported ${waypoints.length - numWaypoints} waypoints.`,
                  'success'
                );
              }
            }}
          >
            <FontAwesomeIcon icon={faDownload} />
            &nbsp;&nbsp;Import
          </Button>
          <Button
            className={styles['flex-grow']}
            tooltip='Export waypoints to text.'
            onClick={() => {
              const text = exportWaypointsAsText();
              navigator.clipboard.writeText(text);
              alertsRef.current?.pushAlert(
                'Waypoints were copied to clipboard.',
                'success'
              );
            }}
            disabled={waypoints.length === 0}
          >
            <FontAwesomeIcon icon={faUpload} />
            &nbsp;&nbsp;Export
          </Button>
        </div>
      </div>
    </div>
  );
}
