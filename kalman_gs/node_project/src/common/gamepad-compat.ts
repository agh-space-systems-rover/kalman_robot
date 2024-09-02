type GamepadInput =
  | 'left-x' // right is positive
  | 'left-y' // up is positive
  | 'right-x'
  | 'right-y'
  | 'left-trigger'
  | 'right-trigger'
  | 'left-shoulder'
  | 'right-shoulder'
  | 'left-stick'
  | 'right-stick'
  | 'a-button'
  | 'b-button'
  | 'x-button'
  | 'y-button'
  | 'dpad-up'
  | 'dpad-down'
  | 'dpad-left'
  | 'dpad-right'
  | 'mode'
  | 'select'
  | 'start';

type GamepadInputMap = {
  [input in GamepadInput]: {
    index: number;
    axis?: boolean;
    remap?: (value: number) => number;
  };
};

const xInputFirefox: GamepadInputMap = {
  'left-x': { index: 0, axis: true },
  'left-y': { index: 1, axis: true, remap: v => -v },
  'right-x': { index: 2, axis: true },
  'right-y': { index: 3, axis: true, remap: v => -v },
  'left-trigger': { index: 4, axis: true, remap: v => v * 0.5 + 0.5 },
  'right-trigger': { index: 5, axis: true, remap: v => v * 0.5 + 0.5 },
  'left-shoulder': { index: 4 },
  'right-shoulder': { index: 5 },
  'left-stick': { index: 10 },
  'right-stick': { index: 11 },
  'a-button': { index: 0 },
  'b-button': { index: 1 },
  'x-button': { index: 3 },
  'y-button': { index: 2 },
  'dpad-up': { index: 12 },
  'dpad-down': { index: 13 },
  'dpad-left': { index: 14 },
  'dpad-right': { index: 15 },
  'mode': { index: 16 },
  'select': { index: 8 },
  'start': { index: 9 },
};
const directInputFirefox = {
  'left-x': { index: 0, axis: true },
  'left-y': { index: 1, axis: true, remap: v => -v },
  'right-x': { index: 4, axis: true },
  'right-y': { index: 5, axis: true, remap: v => -v },
  'left-trigger': { index: 4 },
  'right-trigger': { index: 5 },
  'left-shoulder': { index: 2 },
  'right-shoulder': { index: 18 },
  'left-stick': { index: 8 },
  'right-stick': { index: 9 },
  'a-button': { index: 1 },
  'b-button': { index: 17 },
  'x-button': { index: 0 },
  'y-button': { index: 3 },
  'dpad-up': { index: 12 },
  'dpad-down': { index: 13 },
  'dpad-left': { index: 14 },
  'dpad-right': { index: 15 },
  'mode': { index: 16 },
  'select': { index: 6 },
  'start': { index: 7 },
  
};
const inputChromium: GamepadInputMap = {
  ...xInputFirefox,
  'left-trigger': { index: 6 },
  'right-trigger': { index: 7 },
  'x-button': xInputFirefox['y-button'],
  'y-button': xInputFirefox['x-button']
};
const inititlizedTriggers = new Map<Gamepad, Map<GamepadInput, boolean>>();
function readGamepad(pad: Gamepad, input: GamepadInput): number {
  // Determine XInput or DirectInput.
  // XInput has 17 buttons, DirectInput differs between browsers.
  const xInput = pad.buttons.length === 17;
  // Determine Firefox or Chromium.
  // On chromium there are 4 axes instead of 6 in both XInput and DirectInput.
  const chromium = pad.axes.length === 4;
  
  // Choose the right mapping for the platform.
  let mapping: GamepadInputMap;
  if (chromium) {
    mapping = inputChromium;
  } else {
    mapping = xInput ? xInputFirefox : directInputFirefox;
  }
  
  // Read and remap the value.
  const { index, axis, remap } = mapping[input];
  let value;
  if (axis) {
    value = pad.axes[index];
  } else {
    value = pad.buttons[index]?.value || 0;
  }
  if (remap) {
    value = remap(value);
  }

  // Set triggers to zero until they were moved from 0.5.
  // Fixes a Firefox XInput bug.
  if (input === 'left-trigger' || input === 'right-trigger') {
    if (!inititlizedTriggers.has(pad)) {
      inititlizedTriggers.set(pad, new Map());
    }
    if (value != 0.5) {
      inititlizedTriggers.get(pad).set(input, true);
    }
    if (!inititlizedTriggers.get(pad).get(input)) {
      value = 0;
    }
  }

  // Apply small deadzone to sticks.
  if (input === 'left-x' || input === 'left-y' || input === 'right-x' || input === 'right-y' || input === 'left-trigger' || input === 'right-trigger') {
    if (Math.abs(value) < 0.1) {
      value = 0;
    }
  }

  return value;
}

export { GamepadInput, readGamepad };
