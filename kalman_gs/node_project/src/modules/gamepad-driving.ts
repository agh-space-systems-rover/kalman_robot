// import { ros } from './ros';
// import { Topic } from 'roslib';
// import { Twist } from './ros.interfaces';

// type DrivingMode = 'forwards' | 'sideways' | 'turn-in-place';

// let forwardsTurnRadius = 1.0;
// const SIDEWAYS_TURN_RADIUS = 5.0;
// const RATE = 10.0;

// type ForwardsDrivingInput = {
//   x: number; // positive = forward translation; m/s
//   y: number; // positive = left translation; m/s
//   turn: number; // positive = left turn (at x=1, y=0); [-1, 1]
// }

// type SidewaysDrivingInput = {
//   y: number; // positive = left translation; m/s
//   turn: number; // positive = backward turn (at y=1); [-1, 1]
// }

// type TurnInPlaceDrivingInput = {
//   z: number; // positive = left; rad/s
// }

// let mode: DrivingMode = 'forwards';
// let input: ForwardsDrivingInput | SidewaysDrivingInput | TurnInPlaceDrivingInput = {
//   x: 0,
//   y: 0,
//   turn: 0
// };

// window.addEventListener('ros-connect', () => {
//   const cmdVel = new Topic({
//     ros: ros,
//     name: '/cmd_vel',
//     messageType: 'geometry_msgs/Twist'
//   });

//   setInterval(() => {
//     const twist: Twist = {
//       linear: { x: 0, y: 0, z: 0 },
//       angular: { x: 0, y: 0, z: 0 }
//     };

//     switch (mode) {
//       case 'forwards': {
//         const { x, y, turn } = input as ForwardsDrivingInput;
        
//         twist.linear.x = x;
//         twist.linear.y = y;
//         // radius = RADIUS / turn
//         // angular = linear / radius = linear * turn / RADIUS
//         twist.angular.z = Math.sqrt(x * x + y * y) * turn / forwardsTurnRadius;
//         break;
//       }
//       case 'sideways': {
//         const { y, turn } = input as SidewaysDrivingInput;
//         twist.linear.y = y;
//         twist.angular.z = y * turn / SIDEWAYS_TURN_RADIUS;
//         break;
//       }
//       case 'turn-in-place': {
//         const { z } = input as TurnInPlaceDrivingInput;
//         twist.angular.z = z;
//         break;
//       }
//     }

//     cmdVel.publish(twist);
//   }, 1000 / RATE);
// });

// export const driveForwards = (x: number, y: number, turn: number) => {
//   mode = 'forwards';
//   input = { x, y, turn };
// }

// export const driveSideways = (y: number, turn: number) => {
//   mode = 'sideways';
//   input = { y, turn };
// }

// export const turnInPlace = (z: number) => {
//   mode = 'turn-in-place';
//   input = { z };
// }

// export const setForwardsTurnRadius = (radius: number) => {
//   forwardsTurnRadius = radius;
// }
