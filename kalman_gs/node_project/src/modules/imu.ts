import { Quaternion, zeroQuat } from '../mini-math-lib';
import { ros } from './ros';
import { Imu } from './ros.interfaces';
import { Topic } from 'roslib';

const imuRotation: Quaternion = Object.assign({}, zeroQuat);
window.addEventListener('ros-connect', () => {
  const imu = new Topic({
    ros: ros,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu'
  });

  imu.subscribe((msg: Imu) => {
    imuRotation.w = msg.orientation.w;
    imuRotation.v.x = msg.orientation.x;
    imuRotation.v.y = msg.orientation.y;
    imuRotation.v.z = msg.orientation.z;
    window.dispatchEvent(new Event('imu-update'));
  });
});

export { imuRotation };
