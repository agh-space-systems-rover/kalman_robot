import { ros } from './ros';
import { NavSatFix } from './ros.interfaces';
import { Topic } from 'roslib';

export type GpsFix = {
  latitude?: number;
  longitude?: number;
};

const gpsFix: GpsFix = {
  latitude: undefined,
  longitude: undefined
};
window.addEventListener('ros-connect', () => {
  const imu = new Topic({
    ros: ros,
    name: '/gps/fix',
    messageType: 'sensor_msgs/NavSatFix'
  });

  imu.subscribe((msg: NavSatFix) => {
    if (msg.status.status < 0) {
      return;
    }
    gpsFix.latitude = msg.latitude;
    gpsFix.longitude = msg.longitude;
    window.dispatchEvent(new Event('gps-update'));
  });
});

export { gpsFix };
