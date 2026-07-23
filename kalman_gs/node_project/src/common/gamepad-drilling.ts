import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;
const MAX_SPEED = 100;
const BUTTON_THRESHOLD = 0.5;

enum DrillState {
  Stop = 0,
  DrillingSite1 = 1,
  DrillingSite2 = 2,
  Home = 3,
  ClosingTubesSite1 = 4,
  ClosingTubesSite2 = 5,
  CleaningDrill = 6,
  OpeningTubesSite1 = 7,
  OpeningTubesSite2 = 8,
  OpeningTubesBothSites = 9
}

type DrillGamepadUpdate = {
  bridgeB?: number;
  bridgeC?: number;
  state?: DrillState;
};

const STATE_BUTTONS = [
  { input: 'b-button', state: DrillState.Stop },
  { input: 'y-button', state: DrillState.Home },
  { input: 'a-button', state: DrillState.CleaningDrill },
  { input: 'x-button', state: DrillState.OpeningTubesBothSites },
  { input: 'left-shoulder', state: DrillState.DrillingSite1 },
  { input: 'left-trigger', state: DrillState.ClosingTubesSite1 },
  { input: 'left-stick', state: DrillState.OpeningTubesSite1 },
  { input: 'right-shoulder', state: DrillState.DrillingSite2 },
  { input: 'right-trigger', state: DrillState.ClosingTubesSite2 },
  { input: 'right-stick', state: DrillState.OpeningTubesSite2 }
] as const;

const clampSpeed = (value: number) => Math.max(-MAX_SPEED, Math.min(MAX_SPEED, Math.round(value * MAX_SPEED)));
const isPressed = (value: number) => value > BUTTON_THRESHOLD;

const readDpadBridge = (positiveButton: 'dpad-up' | 'dpad-right', negativeButton: 'dpad-down' | 'dpad-left') => {
  const positivePressed = readGamepads(positiveButton, 'drill') > 0;
  const negativePressed = readGamepads(negativeButton, 'drill') > 0;

  if (positivePressed === negativePressed) return undefined;
  return positivePressed ? MAX_SPEED : -MAX_SPEED;
};

const dispatchDrillGamepadUpdate = (detail: DrillGamepadUpdate) => {
  window.dispatchEvent(new CustomEvent<DrillGamepadUpdate>('drill-gamepad-update', { detail }));
};

window.addEventListener('ros-connect', () => {
  const drillBTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/b',
    messageType: 'std_msgs/Int8'
  });
  const drillCTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/c',
    messageType: 'std_msgs/Int8'
  });
  const drillStateTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/state',
    messageType: 'std_msgs/UInt8'
  });

  let lastBridgeB = 0;
  let lastBridgeC = 0;
  const lastStateButtonValues = new Map<(typeof STATE_BUTTONS)[number]['input'], number>();

  setInterval(() => {
    const bridgeB = readDpadBridge('dpad-up', 'dpad-down') ?? clampSpeed(readGamepads('left-y', 'drill'));
    const bridgeC = readDpadBridge('dpad-right', 'dpad-left') ?? clampSpeed(readGamepads('left-x', 'drill'));

    if (bridgeB !== lastBridgeB) {
      drillBTopic.publish({ data: bridgeB });
      dispatchDrillGamepadUpdate({ bridgeB });
      lastBridgeB = bridgeB;
    }
    if (bridgeC !== lastBridgeC) {
      drillCTopic.publish({ data: bridgeC });
      dispatchDrillGamepadUpdate({ bridgeC });
      lastBridgeC = bridgeC;
    }

    const stateButtonValues = STATE_BUTTONS.map(({ input, state }) => ({
      input,
      state,
      value: readGamepads(input, 'drill'),
      lastValue: lastStateButtonValues.get(input) ?? 0
    }));
    const requestedState = stateButtonValues.find(({ value, lastValue }) => isPressed(value) && !isPressed(lastValue));

    if (requestedState !== undefined) {
      drillStateTopic.publish({ data: requestedState.state });
      dispatchDrillGamepadUpdate({ state: requestedState.state });
    }

    for (const { input, value } of stateButtonValues) {
      lastStateButtonValues.set(input, value);
    }
  }, 1000 / RATE);
});
