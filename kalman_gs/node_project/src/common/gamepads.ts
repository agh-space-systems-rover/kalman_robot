import { GamepadInput, readGamepad } from './gamepad-compat';

type GamepadMode = 'none' | 'wheels' | 'arm' | 'drill';
const gamepadModes: GamepadMode[] = ['none', 'wheels', 'arm', 'drill'];

type GamepadEntry = {
  pad: Gamepad;
  mode: GamepadMode;
};

const gamepads: Map<string, GamepadEntry> = new Map();
const oldModeButtonState: Map<string, number> = new Map();

const ARDUINO_GAMEPAD_ID = '2341-8036-Arduino LLC Arduino Leonardo';
const defaultMode: GamepadMode = 'none';

const connectGamepads = () => {
  const connectedGamepads = navigator.getGamepads().filter((pad) => pad !== null);

  // Handle gamepad actions
  for (const pad of connectedGamepads) {
    // Do not handle Arduino custom controller
    if (pad.id === ARDUINO_GAMEPAD_ID) continue;

    // Add new controller or update its reference
    if (!gamepads.has(pad.id)) {
      gamepads.set(pad.id, { pad, mode: defaultMode });
    } else {
      gamepads.get(pad.id).pad = pad;
    }

    // Get entry
    // Then handle gamepad actions
    const entry = gamepads.get(pad.id)!;

    if (readGamepad(pad, 'select')) {
      entry.mode = 'arm';
    }

    if (readGamepad(pad, 'start')) {
      entry.mode = 'wheels';
    }

    if (readGamepad(pad, 'mode') && !oldModeButtonState.get(pad.id)) {
      const nextIndex = (gamepadModes.indexOf(entry.mode) + 1) % gamepadModes.length;
      entry.mode = gamepadModes[nextIndex];
    }

    oldModeButtonState.set(pad.id, readGamepad(pad, 'mode'));
  }

  // Remove disconnected gamepads.
  for (const id of gamepads.keys()) {
    if (!connectedGamepads.some((pad) => pad.id === id)) {
      gamepads.delete(id);
      oldModeButtonState.delete(id);
    }
  }
  window.dispatchEvent(new Event('gamepads-connect'));
};

window.addEventListener('gamepadconnected', connectGamepads);
window.addEventListener('gamepaddisconnected', connectGamepads);
setInterval(connectGamepads, 100);

function setGamepadMode(pad: Gamepad, mode: GamepadMode) {
  gamepads.get(pad.id).mode = mode;
  window.dispatchEvent(new Event('gamepads-connect'));
}

function readGamepads(input: GamepadInput, padMode: GamepadMode): number {
  let value = 0;
  let numPads = 0;
  for (const { pad, mode } of gamepads.values()) {
    if (mode !== padMode) {
      continue;
    }
    value += readGamepad(pad, input);
    numPads++;
  }
  return value / Math.max(numPads, 1);
}

export { gamepads, GamepadMode, setGamepadMode, readGamepads };
