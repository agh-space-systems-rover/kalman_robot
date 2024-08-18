import { alertsRef, settingsRef } from '../common/refs';
import { ros } from '../common/ros';
import { SetFeedRequest } from '../common/ros-interfaces';
import { getKeybind } from './keybinds';
import { Service, Topic } from 'roslib';

let setLinearScale: Topic<unknown> | null = null;
let setRotationalScale: Topic<unknown> | null = null;

let lastServoLinearScale: number | null = 0.5;
let lastServoRotationalScale: number | null = 0.5;

// ROS
window.addEventListener('ros-connect', () => {
  setLinearScale = new Topic({
    ros: ros,
    name: '/servo/set_linear_scale',
    messageType: 'std_msgs/Float32'
  });

  setRotationalScale = new Topic({
    ros: ros,
    name: '/servo/set_rotational_scale',
    messageType: 'std_msgs/Float32'
  });

  setLinearScale.subscribe((msg: { data: number }) => {
    lastServoLinearScale = msg.data;
    window.dispatchEvent(new Event('servo-linear-scale'));
  });

  setRotationalScale.subscribe((msg: { data: number }) => {
    lastServoRotationalScale = msg.data;
    window.dispatchEvent(new Event('servo-rotational-scale'));
  });

  setLinearScaleTo(lastServoLinearScale);
  setRotationalScaleTo(lastServoRotationalScale);
});

function setLinearScaleTo(value: number) {
  if (setLinearScale) {
    setLinearScale.publish({ data: value });
  }
}

function setRotationalScaleTo(value: number) {
  if (setRotationalScale) {
    setRotationalScale.publish({ data: value });
  }
}

window.addEventListener('keydown', (event) => {
  // Check if any input box is focused.
  if (document.activeElement.tagName === 'INPUT') {
    return;
  }
  // Check if settings are open.
  if (settingsRef.current?.isShown()) {
    return;
  }

  switch (event.code) {
    case getKeybind('Set Spacemouse Linear Scale to Slow (0.1)'):
      setLinearScaleTo(0.1);
      break;
    case getKeybind('Set Spacemouse Linear Scale to Medium (0.3)'):
      setLinearScaleTo(0.3);
      break;
    case getKeybind('Set Spacemouse Linear Scale to Fast (0.7)'):
      setLinearScaleTo(0.7);
      break;
    case getKeybind('Set Spacemouse Rotational Scale to Slow (0.1)'):
      setRotationalScaleTo(0.1);
      break;
    case getKeybind('Set Spacemouse Rotational Scale to Medium (0.3)'):
      setRotationalScaleTo(0.3);
      break;
    case getKeybind('Set Spacemouse Rotational Scale to Fast (0.7)'):
      setRotationalScaleTo(0.7);
      break;
  }
});

export {
  setLinearScaleTo,
  setRotationalScaleTo,
  lastServoLinearScale,
  lastServoRotationalScale
};
