import { ros } from './ros';
import { NavSatFix } from './ros-interfaces';
import { Topic } from 'roslib';

export type GpsCoords = {
  latitude?: number;
  longitude?: number;
};

// Switch to raw gps fix after this many ms of no response from Kalman filter:
const FILTER_TIMEOUT = 2000
let lastKalmanFilterFixTime = 0;
// This prevents rover marker from jumping around when two slightly different
// GPS fixes (in different frame IDs) are received alternately from two sources.

const gpsCoords: GpsCoords = {
  latitude: undefined,
  longitude: undefined
};
window.addEventListener('ros-connect', () => {
  // Subscribe raw sensor data to work without any odometry systems.
  const gps = new Topic({
    ros: ros,
    name: '/gps/fix',
    messageType: 'sensor_msgs/NavSatFix'
  });
  // And similarly, we can subscribe GPS fix from the Kalman filter
  // navsat_transform_node publishes that fix after it receives fused odometry.
  // Though we will only get that topic if our kalman_slam package is launched.
  const gpsFiltered = new Topic({
    ros: ros,
    name: '/gps/filtered',
    messageType: 'sensor_msgs/NavSatFix'
  });

  const gpsCb = (msg: NavSatFix) => {
    if (msg.status.status < 0) {
      return;
    }
    gpsCoords.latitude = msg.latitude;
    gpsCoords.longitude = msg.longitude;
    window.dispatchEvent(new Event('gps-update'));
  };

  gps.subscribe((msg: NavSatFix) => {
    if (Date.now() - lastKalmanFilterFixTime > FILTER_TIMEOUT) {
      gpsCb(msg);
    }
  });
  gpsFiltered.subscribe((msg: NavSatFix) => {
    lastKalmanFilterFixTime = Date.now();
    gpsCb(msg);
  });
});

export { gpsCoords };
