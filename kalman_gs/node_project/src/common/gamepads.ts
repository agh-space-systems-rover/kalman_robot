import { GamepadInput, readGamepad } from "./gamepad-compat";

type GamepadMode = 'wheels' | 'arm';
const gamepads: Map<Gamepad, GamepadMode> = new Map();

const connectGamepads = () => {
  let defaultMode: GamepadMode = 'wheels';
  if (gamepads.size > 0) {
    defaultMode = gamepads.values().next().value === 'wheels' ? 'arm' : 'wheels';
  }

  const connectedGamepads = navigator
    .getGamepads()
    .filter((pad) => pad !== null);
  // Add new gamepads.
  for (const pad of connectedGamepads) {
    if (!gamepads.has(pad)) {
      gamepads.set(pad, defaultMode);
    }
  }
  // Remove disconnected gamepads.
  for (const pad of gamepads.keys()) {
    if (!connectedGamepads.includes(pad)) {
      gamepads.delete(pad);
    }
  }
  window.dispatchEvent(new Event('gamepads-connect'));
};

window.addEventListener('gamepadconnected', connectGamepads);
window.addEventListener('gamepaddisconnected', connectGamepads);
setInterval(connectGamepads, 100);

function setGamepadMode(pad: Gamepad, mode: GamepadMode) {
  gamepads.set(pad, mode);
  window.dispatchEvent(new Event('gamepads-connect'));
}

function readGamepads(input: GamepadInput, mode: GamepadMode): number {
  let value = 0;
  let numPads = 0;
  for (const [pad, padMode] of gamepads) {
    if (padMode !== mode) {
      continue;
    }
    value += readGamepad(pad, input);
    numPads++;
  }
  return value / numPads;
}

export {
  gamepads,
  GamepadMode,
  setGamepadMode,
  readGamepads
};
