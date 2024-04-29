import styles from './wheels.module.css';

import kalmanBody from '!!url-loader!../media/kalman-body.svg';
import kalmanLeftWheel from '!!url-loader!../media/kalman-wheel.svg';
import { ros } from '../modules/ros';
import { Topic } from 'roslib';
import { WheelStates } from '../modules/ros.interfaces';
import { useCallback, useEffect, useState } from 'react';

let lastWheelStates: WheelStates | null = null;
window.addEventListener('ros-connect', () => {
  const imu = new Topic({
    ros: ros,
    name: '/wheel_states/return',
    messageType: 'kalman_interfaces/WheelStates',
  });

  imu.subscribe((msg: WheelStates) => {
    lastWheelStates = msg;
    window.dispatchEvent(new Event('wheel-states-return'));
  });
});

type ArrowProps = {
  velocity: number;
  angle: number;
};

function Arrow({
  velocity,
  angle,
}: ArrowProps) {
  const opacity = Math.pow(Math.min(1, Math.abs(velocity)), 0.25);
  const scale = Math.sign(velocity) * Math.pow(Math.min(1, Math.max(0, Math.abs(velocity))), 0.5);

  return (
    <div
      className={styles['arrow']}
      style={{
        width: 50,
        transform: `rotate(${angle - 90}deg) scale(${scale})`,
        opacity,
      }}
    />
  );
}

type WheelProps = {
  type: 'fl' | 'fr' | 'bl' | 'br';
  angle: number;
  velocity: number;
};

function Wheel({
  type,
  angle,
  velocity
}: WheelProps) {
  return (
    <div
      className={styles['wheel'] + ' ' + styles[type]}
      style={{
        transform: `rotate(${(-angle * 180 / Math.PI) + (type === 'fr' || type === 'br' ? 180 : 0)}deg)`,
      }}
    >
      <img
        src={kalmanLeftWheel}
        className={styles['wheel-image']}
      />
      <Arrow velocity={velocity} angle={(type === 'fr' || type === 'br' ? 180 : 0)} />
    </div>
  );
}

function Rover() {
  const [rerenderCount, setRerenderCount] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCount(count => count + 1);
  }, [setRerenderCount]);
  
  useEffect(() => {
    window.addEventListener('wheel-states-return', rerender);
    return () => {
      window.removeEventListener('wheel-states-return', rerender);
    };
  }, [rerender]);
    

  return (
    <div className={styles['rover']}>
      <img src={kalmanBody} className={styles['body']} />
      <Wheel type="fl" angle={lastWheelStates?.front_left.angle || 0} velocity={lastWheelStates?.front_left.velocity || 0} />
      <Wheel type="fr" angle={lastWheelStates?.front_right.angle || 0} velocity={lastWheelStates?.front_right.velocity || 0} />
      <Wheel type="bl" angle={lastWheelStates?.back_left.angle || 0} velocity={lastWheelStates?.back_left.velocity || 0} />
      <Wheel type="br" angle={lastWheelStates?.back_right.angle || 0} velocity={lastWheelStates?.back_right.velocity || 0} />
    </div>
  );
}

export default function Wheels() {
  return (
    <div className={styles['wheels']}>
      <Rover />
    </div>
  );
}
