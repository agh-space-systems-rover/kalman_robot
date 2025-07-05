import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Drive } from './ros-interfaces';
import { Topic } from 'roslib';

const RATE = 30;
const SPEED = 1.0;
const SPEED_FACTOR_RATE_OF_CHANGE = 0.5;
const MIN_SPEED_FACTOR = 0.2;
const TURN_RADIUS = 0.75;
const ROTATE_IN_PLACE_SPEED = 1.57;
const TRUMPET_LEFT = new Audio('bike_horn.mp3');
const TRUMPET_RIGHT = new Audio('bells.mp3');

let lastDrive: Drive = null;
let speedFactor = MIN_SPEED_FACTOR;
let trumpetLeft = 0;
let trumpetRight = 0;

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
    let changeSpeed = 0; // Y - X

    speed = readGamepads('right-trigger', 'wheels') - readGamepads('left-trigger', 'wheels');
    turn = readGamepads('left-x', 'wheels');
    angle = readGamepads('right-x', 'wheels');
    shoulder = readGamepads('left-shoulder', 'wheels') > 0 || readGamepads('right-shoulder', 'wheels') > 0;
    changeSpeed = readGamepads('y-button', 'wheels') - readGamepads('x-button', 'wheels');

    if (readGamepads('left-stick', 'wheels') == 1 && trumpetLeft == 0) {
      TRUMPET_LEFT.play();
    }
    trumpetLeft = readGamepads('left-stick', 'wheels');

    if (readGamepads('right-stick', 'wheels') == 1 && trumpetRight == 0) {
      TRUMPET_RIGHT.play();
    }
    trumpetRight = readGamepads('right-stick', 'wheels');

    speedFactor += (changeSpeed * SPEED_FACTOR_RATE_OF_CHANGE) / RATE;
    speedFactor = Math.max(MIN_SPEED_FACTOR, Math.min(1, speedFactor));
    speed *= speedFactor;

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
