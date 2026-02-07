import { quatConj, Quaternion, quatFromAxisAngle, quatTimesQuat, zeroQuat } from './mini-math-lib';
import { ros } from './ros';
import { Imu, Odometry, Quaternion as RosQuat } from './ros-interfaces';
import { Topic } from 'roslib';

const FILTER_TIMEOUT = 2000; // switch to raw IMU data after Kalman filter does not respond for this long
let lastKfTime = 0;

const ROS_IMU_LINK_YAW = -1.57; // yaw of imu_link in relation to base_link = the yaw transform from imu_link frame to base_link frame
function imuLinkToBaseLink(orientation: Quaternion): Quaternion {
  const imuToWorld = Object.assign({}, orientation);
  const imuToBase = quatFromAxisAngle({ x: 0, y: 0, z: 1 }, ROS_IMU_LINK_YAW);
  const baseToImu = quatConj(imuToBase);
  const baseToMap = quatTimesQuat(imuToWorld, baseToImu);
  return baseToMap;
}

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

  const imuCb = (orientation: RosQuat, frame: string) => {
    imuRotation.w = orientation.w;
    imuRotation.v.x = orientation.x;
    imuRotation.v.y = orientation.y;
    imuRotation.v.z = orientation.z;
    if (frame === 'imu_link') {
      Object.assign(imuRotation, imuLinkToBaseLink(imuRotation));
    }
    window.dispatchEvent(new CustomEvent('imu-update'));
  };

  imu.subscribe((msg: Imu) => {
    if (Date.now() - lastKfTime > FILTER_TIMEOUT) {
      imuCb(msg.orientation, 'imu_link');
    }
  });
  odom.subscribe((msg: Odometry) => {
    lastKfTime = Date.now();
    imuCb(msg.pose.pose.orientation, 'base_link');
  });
});

export { imuRotation };
