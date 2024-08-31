import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Drive } from './ros-interfaces';
import { Topic } from 'roslib';

const RATE = 30;
const SPEED = 1.0;
const TURN_RADIUS = 0.75;
const ROTATE_IN_PLACE_SPEED = 1.57;

let lastDrive: Drive = null;

window.addEventListener('ros-connect', () => {
  const drive = new Topic<Drive>({
    ros: ros,
    name: '/drive',
    messageType: 'kalman_interfaces/Drive'
  });

  setInterval(() => {
    let speed = 0; // RT - LT
    let turn = 0; // LX
    let angle = 0; // RX
    let shoulder = false; // RB/LB

    speed = readGamepads('right-trigger', 'wheels') - readGamepads('left-trigger', 'wheels');
    turn = readGamepads('left-x', 'wheels');
    angle = readGamepads('right-x', 'wheels');
    shoulder = readGamepads('left-shoulder', 'wheels') > 0 || readGamepads('right-shoulder', 'wheels') > 0;

    const msg: Drive = {
      speed: speed * SPEED,
      inv_radius: -turn / TURN_RADIUS,
      sin_angle: -angle,
      rotation: shoulder ? -speed * ROTATE_IN_PLACE_SPEED : 0
    };
    if (shoulder && msg.rotation === 0) {
      msg.rotation = 0.0001;
    }

    if (msg.speed === 0 && msg.inv_radius === 0 && msg.sin_angle === 0 && msg.rotation === 0) {
      if (JSON.stringify(lastDrive) === JSON.stringify(msg)) {
        return;
      }
    }

    lastDrive = msg;
    drive.publish(msg);
  }, 1000 / RATE);
});
