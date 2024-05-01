import { ros } from './ros';
import { Topic } from 'roslib';
import { Twist } from './ros.interfaces';

const RATE = 10; // Hz
const INPUT_TIMEOUT = 1000; // Ignore input group after 1 second of inactivity.

type CmdVel = {
  x: number;
  y: number;
  angular: number;
  lastUpdate?: number;
};

type CmdVelInputs = {
  [key: string]: CmdVel;
};

const inputs: CmdVelInputs = {};

let lastNonZeroUpdate: number = 0;
window.addEventListener('ros-connect', () => {
  const cmdVel = new Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  setInterval(() => {
    // Sum input from all groups.
    const input: CmdVel = { x: 0, y: 0, angular: 0 };
    const now: number = new Date().getTime();
    let activeGroups = 0;
    for (const group in inputs) {
      const { x, y, angular, lastUpdate } = inputs[group];
      if (now - lastUpdate < INPUT_TIMEOUT) {
        input.x += x;
        input.y += y;
        input.angular += angular;
        activeGroups++;
      }
    }

    // Stop sending commands if all input groups are inactive.
    if (activeGroups === 0) {
      return;
    }

    // Send command.
    const msg: Twist = {
      linear: {
        x: input.x,
        y: input.y,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: input.angular
      }
    };
    cmdVel.publish(msg);
  }, 1000 / RATE);
});

export const setCmdVel = (x: number, y: number, angular: number, inputGroup: string = 'default') => {
  inputs[inputGroup] = { x, y, angular };
  inputs[inputGroup].lastUpdate = Date.now();
}

export const zeroCmdVel = (inputGroup: string = 'default') => {
  inputs[inputGroup] = { x: 0, y: 0, angular: 0 };
  inputs[inputGroup].lastUpdate = Date.now();
}
