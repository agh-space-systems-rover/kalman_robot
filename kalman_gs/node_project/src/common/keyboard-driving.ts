import { setCmdVel } from './cmd-vel';
import { getKeybind } from './keybinds';
import { clamp } from './mini-math-lib';
import { settingsRef } from './refs';

const RATE = 60;
const STALL_SPEED = 0.001;

let speedI = 1;
let turnRadiusI = 2;
let rotateInPlaceSpeedI = 1;

const SPEEDS = [0.2, 0.35, 0.5, 1.0];
const TURN_RADII = [5.0, 2.0, 1.0, 0.5];
const ROTATE_IN_PLACE_SPEEDS = [0.25, 0.5, 1.0, 2.0];
const TURN_INPUT_MAX_RATE_OF_CHANGE = 15 // keypress/s; Later scaled with turn radius.

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
  const isModifyingSpeeds =
    trackedButtons[getKeybind('Hold to Modify Speeds and Turn Radius')];
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

let rotatingInPlace = false;
let lastInputs = [0, 0, 0];
let smoothTurn = 0;

// Update cmd_vel based on the tracked buttons.
function readKey(key: string): number {
  const isModifyingSpeeds =
    trackedButtons[getKeybind('Hold to Modify Speeds and Turn Radius')];
  if (isModifyingSpeeds) {
    return 0;
  }
  return trackedButtons[key] ? 1 : 0;
}
setInterval(() => {
  const forward =
    readKey(getKeybind('Drive Forward')) -
    readKey(getKeybind('Drive Backward')); // positive = forward
  let turn =
    readKey(getKeybind('Turn Left')) - readKey(getKeybind('Turn Right')); // positive = left
  const rotateInPlace =
    readKey(getKeybind('Rotate Left in Place')) -
    readKey(getKeybind('Rotate Right in Place')); // positive = left

  // Smoothen turn input to prevent wheel controller's safety mechanisms from interfering.
  const maxDeltaTurn = TURN_INPUT_MAX_RATE_OF_CHANGE / RATE * TURN_RADII[turnRadiusI];
  smoothTurn += clamp(
    turn - smoothTurn,
    -maxDeltaTurn,
    maxDeltaTurn
  );
  turn = smoothTurn; // Override turn with the smoothed value so that this block can be easily commented out.

  // If inputs did not change since last update and all of them are 0, stop sending commands.
  if (
    lastInputs[0] === forward &&
    lastInputs[1] === turn &&
    lastInputs[2] === rotateInPlace &&
    forward === 0 &&
    turn === 0 &&
    rotateInPlace === 0
  ) {
    return;
  }
  lastInputs = [forward, turn, rotateInPlace];

  if (rotateInPlace !== 0) {
    rotatingInPlace = true;
  } else if (forward !== 0 || turn !== 0) {
    rotatingInPlace = false;
  }

  if (!rotatingInPlace) {
    let linear = forward * SPEEDS[speedI];
    if (linear === 0) {
      linear = STALL_SPEED;
    }
    // radius = RADIUS / turn
    // angular = linear / radius = linear * turn / RADIUS
    const angular = (linear * turn) / TURN_RADII[turnRadiusI];

    setCmdVel(linear, 0, angular, 'keyboard');
  } else {
    let angular = rotateInPlace * ROTATE_IN_PLACE_SPEEDS[rotateInPlaceSpeedI];
    if (angular === 0) {
      angular = STALL_SPEED;
    }

    setCmdVel(0, 0, angular, 'keyboard');
  }
}, RATE);
