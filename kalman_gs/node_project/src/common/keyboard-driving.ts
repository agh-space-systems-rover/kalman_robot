import { getKeybind } from './keybinds';
import { settingsRef } from './refs';
import { ros } from './ros';
import { Drive } from './ros-interfaces';
import { Topic } from 'roslib';

const RATE = 30;
const SPEED_ACCEL = 2.0;
const ROTATE_ACCEL = 2.0;

let speedI = 1;
let turnRadiusI = 2;
let rotateInPlaceSpeedI = 1;

const SPEEDS = [0.2, 0.35, 0.5, 1.0];
const TURN_RADII = [5.0, 2.0, 1.0, 0.5];
const ROTATE_IN_PLACE_SPEEDS = [0.25, 0.5, 1.0, 2.0];

let lastDrive: Drive = null;
let smoothSpeed = 0;
let smoothRotation = 0;

// Track a set of buttons.
const trackedButtons = {};
window.addEventListener('keydown', (event) => {
  // Check if any input box is focused.
  if (document.activeElement.tagName === 'INPUT') {
    return;
  }
  if (settingsRef.current?.isShown()) {
    return;
  }

  trackedButtons[event.code] = true;
  const isModifyingSpeedsKeybind = getKeybind('Hold to Modify Speeds and Turn Radius');
  const isModifyingSpeeds = isModifyingSpeedsKeybind === null ? false : trackedButtons[isModifyingSpeedsKeybind];
  if (isModifyingSpeeds) {
    if (event.code === getKeybind('Drive Backward')) {
      if (speedI > 0) {
        speedI--;
      }
    } else if (event.code === getKeybind('Drive Forward')) {
      if (speedI < SPEEDS.length - 1) {
        speedI++;
      }
    } else if (event.code === getKeybind('Turn Left')) {
      if (turnRadiusI > 0) {
        turnRadiusI--;
      }
    } else if (event.code === getKeybind('Turn Right')) {
      if (turnRadiusI < TURN_RADII.length - 1) {
        turnRadiusI++;
      }
    } else if (event.code === getKeybind('Rotate Left in Place')) {
      if (rotateInPlaceSpeedI > 0) {
        rotateInPlaceSpeedI--;
      }
    } else if (event.code === getKeybind('Rotate Right in Place')) {
      if (rotateInPlaceSpeedI < ROTATE_IN_PLACE_SPEEDS.length - 1) {
        rotateInPlaceSpeedI++;
      }
    }
  }
});
window.addEventListener('keyup', (event) => {
  trackedButtons[event.code] = false;
});

// Update cmd_vel based on the tracked buttons.
function readKey(key: string): number {
  const isModifyingSpeedsKeybind = getKeybind('Hold to Modify Speeds and Turn Radius');
  const isModifyingSpeeds = isModifyingSpeedsKeybind === null ? false : trackedButtons[isModifyingSpeedsKeybind];
  if (isModifyingSpeeds) {
    return 0;
  }
  return trackedButtons[key] ? 1 : 0;
}

window.addEventListener('ros-connect', () => {
  const drive = new Topic<Drive>({
    ros: ros,
    name: '/drive',
    messageType: 'kalman_interfaces/Drive'
  });

  setInterval(() => {
    const forward =
      readKey(getKeybind('Drive Forward')) -
      readKey(getKeybind('Drive Backward')); // positive = forward
    let turn =
      readKey(getKeybind('Turn Left')) - readKey(getKeybind('Turn Right')); // positive = left
    const rotateInPlace =
      readKey(getKeybind('Rotate Left in Place')) -
      readKey(getKeybind('Rotate Right in Place')); // positive = left

    const msg: Drive = {
      speed: forward * SPEEDS[speedI],
      inv_radius: turn / TURN_RADII[turnRadiusI],
      rotation: rotateInPlace * ROTATE_IN_PLACE_SPEEDS[rotateInPlaceSpeedI]
    };

    let dv = msg.speed - smoothSpeed;
    dv = Math.min(Math.max(dv, -SPEED_ACCEL / RATE), SPEED_ACCEL / RATE);
    smoothSpeed += dv;
    msg.speed = smoothSpeed;

    dv = msg.rotation - smoothRotation;
    dv = Math.min(Math.max(dv, -ROTATE_ACCEL / RATE), ROTATE_ACCEL / RATE);
    smoothRotation += dv;
    msg.rotation = smoothRotation;

    if (msg.speed === 0 && msg.inv_radius === 0 && msg.rotation === 0) {
      if (JSON.stringify(lastDrive) === JSON.stringify(msg)) {
        return;
      }
    }

    lastDrive = msg;
    drive.publish(msg);
  }, 1000 / RATE);
});
