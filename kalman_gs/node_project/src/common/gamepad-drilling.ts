import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;
const MAX_SPEED = 100;
const BUTTON_THRESHOLD = 0.5;

type AutonomyState = 0 | 1 | 2 | 3 | 4 | 5;
type DrillGear = 1 | 2;

type DrillGamepadUpdate = {
  bridgeB?: number;
  bridgeC?: number;
  autonomy?: AutonomyState;
  gear?: DrillGear;
  weightRequest?: 0 | 1;
};

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
  const drillAutonomyTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/autonomy',
    messageType: 'std_msgs/UInt8'
  });
  const drillGearTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/gear',
    messageType: 'std_msgs/UInt8'
  });
  const drillWeightReqTopic = new Topic<{ data: number }>({
    ros,
    name: '/science/drill/weight/request',
    messageType: 'std_msgs/UInt8'
  });

  let lastBridgeB = 0;
  let lastBridgeC = 0;
  let lastStopButton = 0;
  let lastDrillButton = 0;
  let lastHomeButton = 0;
  let lastOpenButton = 0;
  let lastCloseButton = 0;
  let lastCleanButton = 0;
  let lastGear1Button = 0;
  let lastGear2Button = 0;
  let lastTareButton = 0;
  let lastWeighButton = 0;

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

    const stopButton = readGamepads('b-button', 'drill');
    const drillButton = readGamepads('a-button', 'drill');
    const homeButton = readGamepads('y-button', 'drill');
    const openButton = readGamepads('x-button', 'drill');
    const closeButton = readGamepads('left-stick', 'drill');
    const cleanButton = readGamepads('right-stick', 'drill');
    const gear1Button = readGamepads('left-shoulder', 'drill');
    const gear2Button = readGamepads('right-shoulder', 'drill');
    const tareButton = readGamepads('left-trigger', 'drill');
    const weighButton = readGamepads('right-trigger', 'drill');

    if (isPressed(stopButton) && !isPressed(lastStopButton)) {
      drillAutonomyTopic.publish({ data: 0 });
      dispatchDrillGamepadUpdate({ autonomy: 0 });
    }
    if (isPressed(drillButton) && !isPressed(lastDrillButton)) {
      drillAutonomyTopic.publish({ data: 1 });
      dispatchDrillGamepadUpdate({ autonomy: 1 });
    }
    if (isPressed(homeButton) && !isPressed(lastHomeButton)) {
      drillAutonomyTopic.publish({ data: 2 });
      dispatchDrillGamepadUpdate({ autonomy: 2 });
    }
    if (isPressed(openButton) && !isPressed(lastOpenButton)) {
      drillAutonomyTopic.publish({ data: 3 });
      dispatchDrillGamepadUpdate({ autonomy: 3 });
    }
    if (isPressed(closeButton) && !isPressed(lastCloseButton)) {
      drillAutonomyTopic.publish({ data: 4 });
      dispatchDrillGamepadUpdate({ autonomy: 4 });
    }
    if (isPressed(cleanButton) && !isPressed(lastCleanButton)) {
      drillAutonomyTopic.publish({ data: 5 });
      dispatchDrillGamepadUpdate({ autonomy: 5 });
    }

    if (isPressed(gear1Button) && !isPressed(lastGear1Button)) {
      drillGearTopic.publish({ data: 1 });
      dispatchDrillGamepadUpdate({ gear: 1 });
    }
    if (isPressed(gear2Button) && !isPressed(lastGear2Button)) {
      drillGearTopic.publish({ data: 2 });
      dispatchDrillGamepadUpdate({ gear: 2 });
    }

    if (isPressed(tareButton) && !isPressed(lastTareButton)) {
      drillWeightReqTopic.publish({ data: 0 });
      dispatchDrillGamepadUpdate({ weightRequest: 0 });
    }
    if (isPressed(weighButton) && !isPressed(lastWeighButton)) {
      drillWeightReqTopic.publish({ data: 1 });
      dispatchDrillGamepadUpdate({ weightRequest: 1 });
    }

    lastStopButton = stopButton;
    lastDrillButton = drillButton;
    lastHomeButton = homeButton;
    lastOpenButton = openButton;
    lastCloseButton = closeButton;
    lastCleanButton = cleanButton;
    lastGear1Button = gear1Button;
    lastGear2Button = gear2Button;
    lastTareButton = tareButton;
    lastWeighButton = weighButton;
  }, 1000 / RATE);
});
