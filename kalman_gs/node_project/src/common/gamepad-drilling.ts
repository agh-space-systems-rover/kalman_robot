import { readGamepad } from './gamepad-compat';
import { getDrillGamepadSettings } from './gamepad-drilling-settings';
import { gamepads, readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;
const BUTTON_THRESHOLD = 0.5;

enum AutonomyState {
  Stop = 0,
  Start = 1,
  AutonomyStop = 2,
  Home = 3
}

type DrillGamepadUpdate = {
  rack?: number;
  drill?: number;
  autonomy?: AutonomyState;
  weightCommand?: 0 | 1;
};

const AUTONOMY_BUTTONS = [
  { input: 'b-button', state: AutonomyState.Stop },
  { input: 'a-button', state: AutonomyState.Start },
  { input: 'x-button', state: AutonomyState.AutonomyStop },
  { input: 'y-button', state: AutonomyState.Home }
] as const;

const WEIGHT_BUTTONS = [
  { input: 'left-shoulder', command: 0 as const },
  { input: 'right-shoulder', command: 1 as const }
] as const;

const clamp = (value: number, max: number) => Math.max(-max, Math.min(max, Math.round(value * max)));
const isPressed = (value: number) => value > BUTTON_THRESHOLD;

const readDpadAxis = (positivePressed: boolean, negativePressed: boolean, max: number) => {
  if (!positivePressed && !negativePressed) return undefined;
  if (positivePressed && negativePressed) return 0;
  return positivePressed ? max : -max;
};

const readDrillMovement = () => {
  let rack = 0;
  let drill = 0;
  let drillGamepadsCount = 0;

  for (const { pad, mode } of gamepads.values()) {
    if (mode !== 'drill') continue;

    const settings = getDrillGamepadSettings(pad.id);
    const dpadRack = readDpadAxis(
      isPressed(readGamepad(pad, 'dpad-down')),
      isPressed(readGamepad(pad, 'dpad-up')),
      settings.rackArrow
    );
    const dpadDrill = readDpadAxis(
      isPressed(readGamepad(pad, 'dpad-right')),
      isPressed(readGamepad(pad, 'dpad-left')),
      settings.drillArrow
    );

    rack += dpadRack ?? clamp(-readGamepad(pad, 'left-y'), settings.rackStick);
    drill += dpadDrill ?? clamp(readGamepad(pad, 'right-x'), settings.drillStick);
    drillGamepadsCount++;
  }

  return {
    rack: Math.round(rack / Math.max(drillGamepadsCount, 1)),
    drill: Math.round(drill / Math.max(drillGamepadsCount, 1))
  };
};

const dispatchDrillGamepadUpdate = (detail: DrillGamepadUpdate) => {
  window.dispatchEvent(new CustomEvent<DrillGamepadUpdate>('drill-gamepad-update', { detail }));
};

window.addEventListener('ros-connect', () => {
  const rackTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/rack',
    messageType: 'std_msgs/Int8'
  });
  const drillTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/drill',
    messageType: 'std_msgs/Int8'
  });
  const autonomyTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/autonomy',
    messageType: 'std_msgs/UInt8'
  });
  const weightCmdTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/weight/cmd',
    messageType: 'std_msgs/UInt8'
  });

  let lastRack = 0;
  let lastDrill = 0;
  const lastButtonValues = new Map<(typeof AUTONOMY_BUTTONS)[number]['input'], number>();
  const lastWeightButtonValues = new Map<(typeof WEIGHT_BUTTONS)[number]['input'], number>();

  window.setInterval(() => {
    const { rack, drill } = readDrillMovement();

    if (rack !== lastRack) {
      rackTopic.publish({ data: rack });
      dispatchDrillGamepadUpdate({ rack });
      lastRack = rack;
    }
    if (drill !== lastDrill) {
      drillTopic.publish({ data: drill });
      dispatchDrillGamepadUpdate({ drill });
      lastDrill = drill;
    }

    const buttonValues = AUTONOMY_BUTTONS.map(({ input, state }) => ({
      input,
      state,
      value: readGamepads(input, 'drill'),
      lastValue: lastButtonValues.get(input) ?? 0
    }));
    const requestedState = buttonValues.find(({ value, lastValue }) => isPressed(value) && !isPressed(lastValue));

    if (requestedState !== undefined) {
      autonomyTopic.publish({ data: requestedState.state });
      dispatchDrillGamepadUpdate({ autonomy: requestedState.state });
    }

    for (const { input, value } of buttonValues) lastButtonValues.set(input, value);

    const weightButtonValues = WEIGHT_BUTTONS.map(({ input, command }) => ({
      input,
      command,
      value: readGamepads(input, 'drill'),
      lastValue: lastWeightButtonValues.get(input) ?? 0
    }));
    const requestedWeightCommand = weightButtonValues.find(
      ({ value, lastValue }) => isPressed(value) && !isPressed(lastValue)
    );

    if (requestedWeightCommand !== undefined) {
      weightCmdTopic.publish({ data: requestedWeightCommand.command });
      dispatchDrillGamepadUpdate({ weightCommand: requestedWeightCommand.command });
    }

    for (const { input, value } of weightButtonValues) lastWeightButtonValues.set(input, value);
  }, 1000 / RATE);
});
