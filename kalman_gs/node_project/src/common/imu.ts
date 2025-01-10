import { Quaternion, zeroQuat } from './mini-math-lib';
import { ros } from './ros';
import { Imu, Odometry, Quaternion as RosQuat } from './ros-interfaces';
import { Topic } from 'roslib';

const FILTER_TIMEOUT = 2000; // switch to raw IMU data after Kalman filter does not respond for this long
let lastKfTime = 0;

const imuRotation: Quaternion = Object.assign({}, zeroQuat);
window.addEventListener('ros-connect', () => {
  const imu = new Topic({
    ros: ros,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu'
  });
  const odom = new Topic({
    ros: ros,
    name: '/odometry/global',
    messageType: 'nav_msgs/Odometry'
  });

  const imuCb = (orientation: RosQuat) => {
    imuRotation.w = orientation.w;
    imuRotation.v.x = orientation.x;
    imuRotation.v.y = orientation.y;
    imuRotation.v.z = orientation.z;
    window.dispatchEvent(new Event('imu-update'));
  };

  imu.subscribe((msg: Imu) => {
    if (Date.now() - lastKfTime > FILTER_TIMEOUT) {
      imuCb(msg.orientation);
    }
  });
  odom.subscribe((msg: Odometry) => {
    lastKfTime = Date.now();
    imuCb(msg.pose.pose.orientation);
  });
});

export { imuRotation };
