import { readGamepads } from './gamepads';
import { ros } from './ros';
import { Topic } from 'roslib';

const RATE = 10;
const MAX_SPEED = 100;

const clampSpeed = (value: number) => Math.max(-MAX_SPEED, Math.min(MAX_SPEED, Math.round(value * MAX_SPEED)));

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
  let lastTareButton = 0;
  let lastWeighButton = 0;

  setInterval(() => {
    const bridgeB = clampSpeed(readGamepads('right-y', 'drill'));
    const bridgeC = clampSpeed(readGamepads('left-x', 'drill'));

    if (bridgeB !== lastBridgeB) {
      drillBTopic.publish({ data: bridgeB });
      lastBridgeB = bridgeB;
    }
    if (bridgeC !== lastBridgeC) {
      drillCTopic.publish({ data: bridgeC });
      lastBridgeC = bridgeC;
    }

    const stopButton = readGamepads('b-button', 'drill');
    const drillButton = readGamepads('a-button', 'drill');
    const homeButton = readGamepads('y-button', 'drill');
    const tareButton = readGamepads('left-shoulder', 'drill');
    const weighButton = readGamepads('right-shoulder', 'drill');

    if (stopButton > 0 && lastStopButton === 0) {
      drillAutonomyTopic.publish({ data: 0 });
    }
    if (drillButton > 0 && lastDrillButton === 0) {
      drillAutonomyTopic.publish({ data: 1 });
    }
    if (homeButton > 0 && lastHomeButton === 0) {
      drillAutonomyTopic.publish({ data: 2 });
    }

    if (tareButton > 0 && lastTareButton === 0) {
      drillWeightReqTopic.publish({ data: 0 });
    }
    if (weighButton > 0 && lastWeighButton === 0) {
      drillWeightReqTopic.publish({ data: 1 });
    }

    lastStopButton = stopButton;
    lastDrillButton = drillButton;
    lastHomeButton = homeButton;
    lastTareButton = tareButton;
    lastWeighButton = weighButton;
  }, 1000 / RATE);
});
