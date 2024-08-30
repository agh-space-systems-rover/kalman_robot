import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;

// let lastAutonomy = 0;
// let lastSendWeight = 0;


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
    // let autonomousDrilling = readGamepads('x-button', 'drill');
    // let stopAll = readGamepads('y-button', 'drill');
    // let sendWeight = readGamepads('b-button', 'drill');


    if(Number.isNaN(armAxis))
        armAxis = 0;
    if(Number.isNaN(rackAxis))
        rackAxis = 0;
    if(Number.isNaN(drilling))
        drilling = 0;

    const drillData = { arm: armAxis, rack: rackAxis, drill: drilling };
    sendDrillData.publish(drillData);
    console.log("wysrałem ramke: ", drillData);
    
    // const msgAutonomy = { cmd: 72, data: [255] };
    // const msgStop = { cmd: 72, data: [0] };
    // const msgWeight = { cmd: 73, data: [20] }

    // lastAutonomy = autonomousDrilling==1?1:0;
    // lastSendWeight = sendWeight==1?1:0;

  }, 1000 / RATE);
});