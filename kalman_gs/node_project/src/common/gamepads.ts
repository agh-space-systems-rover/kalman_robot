import { audioLoop } from '../indicators/interstellar';
import { alertsRef } from './refs';
import { ros } from './ros';
import { GamepadState, GamepadStates, SetGamepadModeRequest, SetGamepadModeResponse } from './ros-interfaces';
import { Service, Topic } from 'roslib';

// Gamepads are read by kalman_gamepad, the station only displays them and changes their modes.

// Same values as in kalman_interfaces/msg/GamepadState.msg.
enum GamepadType {
  Gamepad = 0,
  Custom = 1,
  SpaceMouse = 2,
  Unsupported = 3
}

enum GamepadMode {
  None = 0,
  Wheels = 1,
  Arm = 2,
  Drill = 3
}

// kalman_gamepad publishes the state every second.
const STATE_TIMEOUT = 3000;

const HORN_LEFT = new Audio('bike_horn.mp3');
const HORN_RIGHT = new Audio('bells.mp3');

let gamepads: GamepadState[] = [];
let lastStateTime = 0;

function updateGamepads(newGamepads: GamepadState[]) {
  for (const pad of gamepads) {
    if (pad.mode !== GamepadMode.None && !newGamepads.some(({ id }) => id === pad.id)) {
      alertsRef.current?.pushAlert(`Lost connection with ${pad.name}.`);
    }
  }
  gamepads = newGamepads;
  window.dispatchEvent(new CustomEvent('gamepads-update'));
}

// Created only once, because roslib subscribes again after reconnecting.
const stateTopic = new Topic<GamepadStates>({
  ros,
  name: '/station/gamepads/state',
  messageType: 'kalman_interfaces/GamepadStates'
});
stateTopic.subscribe((msg) => {
  lastStateTime = Date.now();
  updateGamepads(msg.gamepads);
});

setInterval(() => {
  if (gamepads.length > 0 && Date.now() - lastStateTime > STATE_TIMEOUT) {
    updateGamepads([]);
  }
}, 1000);

const soundTopic = new Topic<{ data: string }>({
  ros,
  name: '/station/gamepads/sound',
  messageType: 'std_msgs/String'
});
soundTopic.subscribe(({ data }) => {
  if (data === 'horn_left') HORN_LEFT.play();
  if (data === 'horn_right') HORN_RIGHT.play();
  if (data === 'interstellar') audioLoop.togglePlayback();
});

const setModeService = new Service<SetGamepadModeRequest, SetGamepadModeResponse>({
  ros,
  name: '/station/gamepads/set_mode',
  serviceType: 'kalman_interfaces/SetGamepadMode'
});

function setGamepadMode(id: string, mode: GamepadMode) {
  if (!ros.isConnected) {
    alertsRef.current?.pushAlert('Failed to set the gamepad mode. ROS connection is not established.');
    return;
  }

  setModeService.callService(
    { id, mode },
    (response) => {
      if (!response.success) {
        alertsRef.current?.pushAlert('Failed to set the gamepad mode. The gamepad was disconnected.');
      }
    },
    (error) => {
      alertsRef.current?.pushAlert(`Failed to set the gamepad mode. ${error}`);
    }
  );
}

export { gamepads, GamepadMode, GamepadType, setGamepadMode };
