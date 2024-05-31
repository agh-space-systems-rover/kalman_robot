import { setCmdVel } from './cmd-vel';

const STALL_SPEED = 0.001;
const ROTATE_IN_PLACE_SPEED = 0.5;

let speedI = 1;
let turnRadiusI = 1;
let lastButtonUpdate: number = 0;

const SPEEDS = [0.2, 0.35, 0.5, 1.0];
const TURN_RADIUSES = [0.5, 1.0, 2.0, 5.0];

// Track a set of buttons.
const TRACKED_BUTTONS = ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE'];
const trackedButtons = {};
window.addEventListener('keydown', (event) => {
  if (TRACKED_BUTTONS.includes(event.code)) {
    trackedButtons[event.code] = true;
  }

  if (event.code === 'ArrowUp') {
    if (speedI < SPEEDS.length - 1) {
      speedI++;
    }
  } else if (event.code === 'ArrowDown') {
    if (speedI > 0) {
      speedI--;
    }
  } else if (event.code === 'ArrowLeft') {
    if (turnRadiusI < TURN_RADIUSES.length - 1) {
      turnRadiusI++;
    }
  } else if (event.code === 'ArrowRight') {
    if (turnRadiusI > 0) {
      turnRadiusI--;
    }
  }
});
window.addEventListener('keyup', (event) => {
  if (TRACKED_BUTTONS.includes(event.code)) {
    trackedButtons[event.code] = false;
  }
});

let rotatingInPlace = false;
let lastInputs = [0, 0, 0];

// Update cmd_vel based on the tracked buttons.
function readKey(key: string): number {
  return trackedButtons[key] ? 1 : 0;
}
setInterval(() => {
  const forward = readKey('KeyW') - readKey('KeyS'); // positive = forward
  const turn = readKey('KeyA') - readKey('KeyD'); // positive = left
  const rotateInPlace = readKey('KeyQ') - readKey('KeyE'); // positive = left

  // If inputs did not change since last update and all of them are 0, stop sending commands.
  if (
    lastInputs[0] === forward &&
    lastInputs[1] === turn &&
    lastInputs[2] === rotateInPlace &&
    forward === 0 &&
    turn === 0 &&
    rotateInPlace === 0
  ) {
    // setCmdVel(0, 0, 0, 'keyboard');
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
    const angular = (linear * turn) / TURN_RADIUSES[turnRadiusI];

    setCmdVel(linear, 0, angular, 'keyboard');
  } else {
    let angular = rotateInPlace * ROTATE_IN_PLACE_SPEED;
    if (angular === 0) {
      angular = STALL_SPEED;
    }

    setCmdVel(0, 0, angular, 'keyboard');
  }
}, 100);
