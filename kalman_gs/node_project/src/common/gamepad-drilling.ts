import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 30;
const SPEED_ARM = 50; // set to -50 to invert
const SPEED_RACK = 50;
const SPEED_DRILL = 100;

let lastArm = {cmd: 69, data: [0, 0]};
let lastRack = {cmd: 70, data: [0, 0]};
let lastDrill = {cmd: 71, data: [0, 0]};
let lastAutonomy = 0;
let lastSendWeight = 0;

function msgsAreEqual(arr1, arr2) {
    if (arr1.data.length !== arr2.data.length) return false;
    for (let i = 0; i < arr1.data.length; i++)
        if (arr1.data[i] !== arr2.data[i]) return false;
    return true;
}

function msgNotZero(msg)
{
    for (let i = 0; i < msg.data.length; i++)
        if (msg.data[i] !== 0) return true;
    return false;
}

window.addEventListener('ros-connect', () => {
  const sendDrillData = new Topic({
    ros: ros,
    name: '/master_com/ros_to_master',
    messageType: 'kalman_interfaces/MasterMessage'
  });

  setInterval(() => {
    let armAxis = readGamepads('left-y', 'drill') * SPEED_ARM;
    let rackAxis = readGamepads('right-y', 'drill') * SPEED_RACK;
    let drilling = readGamepads('right-x', 'drill') * SPEED_DRILL;
    let autonomousDrilling = readGamepads('x-button', 'drill');
    let stopAll = readGamepads('y-button', 'drill');
    let sendWeight = readGamepads('b-button', 'drill');


    if(Number.isNaN(armAxis))
        armAxis = 0;
    if(Number.isNaN(rackAxis))
        rackAxis = 0;
    if(Number.isNaN(drilling))
        drilling = 0;

    const msgArm = { cmd: 69, data: [armAxis<0?1:0, Math.abs(armAxis)] };
    const msgRack = { cmd: 70, data: [rackAxis<0?1:0, Math.abs(rackAxis)] };
    const msgDrill = { cmd: 71, data: [drilling<0?1:0, Math.abs(drilling)] };
    const msgAutonomy = { cmd: 72, data: [255] };
    const msgStop = { cmd: 72, data: [0] };
    const msgWeight = { cmd: 73, data: [20] }

    if(stopAll == 1)
        sendDrillData.publish(msgStop);
    else
    {
        if(sendWeight && !lastSendWeight)
            sendDrillData.publish(msgWeight);
        if(autonomousDrilling && !lastAutonomy)
            sendDrillData.publish(msgAutonomy);
    }

    if(!msgsAreEqual(msgArm, lastArm))
        sendDrillData.publish(msgArm);
    if(!msgsAreEqual(msgRack, lastRack))
        sendDrillData.publish(msgRack);
    if(!msgsAreEqual(msgDrill, lastDrill))
        sendDrillData.publish(msgDrill);

    lastArm = msgArm;
    lastRack = msgRack;
    lastDrill = msgDrill;
    lastAutonomy = autonomousDrilling==1?1:0;
    lastSendWeight = sendWeight==1?1:0;

  }, 1000 / RATE);
});