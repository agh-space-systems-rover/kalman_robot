import styles from './wheels.module.css';

import kalmanBody from '!!url-loader!../media/kalman-body.svg';
import kalmanLeftWheelOutline from '!!url-loader!../media/kalman-wheel-outline.svg';
import kalmanLeftWheel from '!!url-loader!../media/kalman-wheel.svg';
import { ros } from '../common/ros';
import { WheelStates, WheelTemperatures } from '../common/ros-interfaces';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

let lastWheelStates: WheelStates | null = null;
let lastWheelStatesReturn: WheelStates | null = null;
let lastWheelTemperatures: WheelTemperatures | null = null;
window.addEventListener('ros-connect', () => {
  const stateTopic = new Topic({
    ros: ros,
    name: '/wheel_states',
    messageType: 'kalman_interfaces/WheelStates'
  });

  stateTopic.subscribe((msg: WheelStates) => {
    lastWheelStates = msg;
    window.dispatchEvent(new Event('wheel-states'));
  });

  const stateReturnTopic = new Topic({
    ros: ros,
    name: '/wheel_states/return',
    messageType: 'kalman_interfaces/WheelStates'
  });

  stateReturnTopic.subscribe((msg: WheelStates) => {
    lastWheelStatesReturn = msg;
    window.dispatchEvent(new Event('wheel-states-return'));
  });

  const temperatureTopic = new Topic({
    ros: ros,
    name: '/wheel_temps',
    messageType: 'kalman_interfaces/WheelTemperatures'
  });

  temperatureTopic.subscribe((msg: WheelTemperatures) => {
    console.log('Received temperatures:' + msg);
    lastWheelTemperatures = msg;
    window.dispatchEvent(new Event('wheel-temps'));
  });
});

type ArrowProps = {
  velocity: number;
  angle: number;
  thickness: number;
};

function Arrow({ velocity, angle, thickness }: ArrowProps) {
  const opacity = Math.pow(Math.min(1, Math.abs(velocity)), 0.25);
  const scale = Math.sign(velocity) * Math.pow(Math.min(1, Math.max(0, Math.abs(velocity))), 0.5);

  return (
    <div
      className={styles['arrow']}
      style={{
        transform: `rotate(${angle - 90}deg) scale(${scale})`,
        opacity,
        borderWidth: thickness * 0.5,
        left: `calc(48% - ${thickness * 0.5}px)`,
        top: `calc(50% - ${thickness * 0.5}px)`,
        transformOrigin: `${thickness * 0.5}px ${thickness * 0.5}px`
      }}
    >
      <div />
      <div
        className={styles['arrow-head']}
        style={{
          borderTopWidth: thickness,
          borderRightWidth: thickness,
          marginRight: -thickness,
          width: thickness * 2,
          height: thickness * 2
        }}
      />
    </div>
  );
}

type WheelProps = {
  type: 'fl' | 'fr' | 'bl' | 'br';
  angle: number;
  velocity: number;
  showTarget: boolean;
};

function Wheel({ type, angle, velocity, showTarget }: WheelProps) {
  const ref = useRef<HTMLDivElement>();
  const thickness = ref.current ? ref.current.clientWidth * 0.3 : 5;

  return (
    <div
      ref={ref}
      className={styles['wheel'] + ' ' + styles[type]}
      style={{
        transform: `rotate(${(-angle * 180) / Math.PI + (type === 'fr' || type === 'br' ? 180 : 0)}deg)`
      }}
    >
      <img
        src={showTarget ? kalmanLeftWheelOutline : kalmanLeftWheel}
        className={styles['wheel-image']}
        draggable='false'
      />
      <Arrow velocity={velocity} angle={type === 'fr' || type === 'br' ? 180 : 0} thickness={thickness} />
    </div>
  );
}

type RoverProps = {
  showTarget?: boolean;
};

function Rover({ showTarget = false }: RoverProps) {
  const [_, setRerenderCount] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, [setRerenderCount]);

  useEffect(() => {
    window.addEventListener('wheel-states', rerender);
    window.addEventListener('wheel-states-return', rerender);
    window.addEventListener('wheel-temps', rerender);
    return () => {
      window.removeEventListener('wheel-states', rerender);
      window.removeEventListener('wheel-states-return', rerender);
      window.removeEventListener('wheel-temps', rerender);
    };
  }, [rerender]);

  const wheelStates = showTarget ? lastWheelStates : lastWheelStatesReturn;
  return (
    <div className={styles['rover'] + (showTarget ? ' show-target' : '')}>
      <div className={styles['rover-h']}>
        <div className={styles['rover-v']}>
          {!showTarget && <img src={kalmanBody} className={styles['body']} draggable='false' />}
          <Wheel
            type='fl'
            angle={wheelStates?.front_left.angle || 0}
            velocity={wheelStates?.front_left.velocity || 0}
            showTarget={showTarget}
          />
          <Wheel
            type='fr'
            angle={wheelStates?.front_right.angle || 0}
            velocity={wheelStates?.front_right.velocity || 0}
            showTarget={showTarget}
          />
          <Wheel
            type='bl'
            angle={wheelStates?.back_left.angle || 0}
            velocity={wheelStates?.back_left.velocity || 0}
            showTarget={showTarget}
          />
          <Wheel
            type='br'
            angle={wheelStates?.back_right.angle || 0}
            velocity={wheelStates?.back_right.velocity || 0}
            showTarget={showTarget}
          />
        </div>
      </div>
    </div>
  );
}

export default function Wheels() {
  return (
    <div className={styles['wheels']}>
      <Rover />
      <Rover showTarget={true} />
    </div>
  );
}
