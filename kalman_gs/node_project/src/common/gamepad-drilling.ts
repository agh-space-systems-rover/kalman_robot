import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;

window.addEventListener('ros-connect', () => {
  const sendDrillData = new Topic({
    ros: ros,
    name: '/drill',
    messageType: 'kalman_interfaces/Drill'
  });

  setInterval(() => {
    let armAxis = readGamepads('left-y', 'drill') ;
    let rackAxis = readGamepads('right-y', 'drill');
    let drilling = readGamepads('right-trigger', 'drill')  - readGamepads('left-trigger', 'drill');

    if(Number.isNaN(armAxis))
        armAxis = 0;
    if(Number.isNaN(rackAxis))
        rackAxis = 0;
    if(Number.isNaN(drilling))
        drilling = 0;

    const drillData = { arm: armAxis, rack: rackAxis, drill: drilling };
    sendDrillData.publish(drillData);
  }, 1000 / RATE);
});