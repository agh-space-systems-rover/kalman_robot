import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;
const SPEED_ARM = 50; // set to -50 to invert
const SPEED_RACK = 50;
const SPEED_DRILL = 100;

let lastArm = {cmd: 69, data: [0, 0]};
let lastRack = {cmd: 70, data: [0, 0]};
let lastDrill = {cmd: 71, data: [0, 0]};
let lastAutonomy = 0;
let lastSendWeight = 0;

let armFiveTimesZero = 0;
let rackFiveTimesZero = 0;
let drillFiveTimesZero = 0;

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
    let drilling = ( readGamepads('right-trigger', 'drill')  - readGamepads('left-trigger', 'drill')) * SPEED_DRILL;
    let autonomousDrilling = readGamepads('x-button', 'drill');
    let stopAll = readGamepads('y-button', 'drill');
    let sendWeight = readGamepads('b-button', 'drill');


    if(Number.isNaN(armAxis))
        armAxis = 0;
    if(Number.isNaN(rackAxis))
        rackAxis = 0;
    if(Number.isNaN(drilling))
        drilling = 0;

    const msgArm = { cmd: 69, data: [armAxis<0?1:0, Math.round( Math.abs(armAxis) )] };
    const msgRack = { cmd: 70, data: [rackAxis<0?1:0, Math.round( Math.abs(rackAxis) )] };
    const msgDrill = { cmd: 71, data: [drilling<0?1:0, Math.round( Math.abs(drilling) )] };
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

    if(msgNotZero(msgArm) || armFiveTimesZero++ < 5)
    {
        sendDrillData.publish(msgArm);
        if(msgNotZero(msgArm))
            armFiveTimesZero = 0;
    }

    if(msgNotZero(msgRack) || rackFiveTimesZero++ < 5)
    {
        sendDrillData.publish(msgRack);
        if(msgNotZero(msgRack))
            rackFiveTimesZero = 0;
    }

    if(msgNotZero(msgDrill) || drillFiveTimesZero++ < 5)
    {
        sendDrillData.publish(msgDrill);
        if(msgNotZero(msgDrill))
            drillFiveTimesZero = 0;
    }


    lastArm = msgArm;
    lastRack = msgRack;
    lastDrill = msgDrill;
    lastAutonomy = autonomousDrilling==1?1:0;
    lastSendWeight = sendWeight==1?1:0;

  }, 1000 / RATE);
});