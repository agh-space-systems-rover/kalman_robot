import { alertsRef } from './refs';
import { Ros } from 'roslib';

const ros = new Ros({
  url: `ws://${window.location.hostname}:9065`
});
// ros.on('connection', () => {
//   console.log('ROSLIB connected.');
// });
ros.on('error', (error) => {
  console.error('ROSLIB error:\n', error, '\nWill attempt to reconnect.');
});

// Schedule an interval to check if the connection was successful.
let lastLoggedConnectionState: boolean | undefined = undefined;
let reconnectionTimeout: NodeJS.Timeout | undefined = undefined;
setInterval(() => {
  if (!ros.isConnected) {
    if (lastLoggedConnectionState === undefined) {
      alertsRef.current?.pushAlert('Failed to connect with ROS.');
    } else if (lastLoggedConnectionState === true) {
      alertsRef.current?.pushAlert('Lost connection with ROS.');
    }
    lastLoggedConnectionState = false;

    if (reconnectionTimeout === undefined) {
      reconnectionTimeout = setTimeout(() => {
        ros.connect('ws://localhost:9065');
        reconnectionTimeout = undefined;
      }, 1000);
    }
  } else if (ros.isConnected) {
    if (lastLoggedConnectionState === undefined) {
      // alertsRef.current?.pushAlert('Connected with ROS.', 'success');
      window.dispatchEvent(new Event('ros-connect'));
    } else if (lastLoggedConnectionState === false) {
      alertsRef.current?.pushAlert(
        'Re-established connection with ROS.',
        'success'
      );
      window.dispatchEvent(new Event('ros-connect'));
    }
    lastLoggedConnectionState = true;
  }
}, 500);

export { ros };
