import styles from './wheels.module.css';

import kalmanBody from '!!url-loader!../media/kalman-body.svg';
import kalmanLeftWheelOutline from '!!url-loader!../media/kalman-wheel-outline.svg';
import kalmanLeftWheel from '!!url-loader!../media/kalman-wheel.svg';
import { ros } from '../common/ros';
import { WheelStates, WheelTemperatures } from '../common/ros-interfaces';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

const KELVIN_OFFSET = 273.15;

// Temperature thresholds:
const MOTOR_TEMPERATURE_WARNING_THRESHOLD = KELVIN_OFFSET + 70; // 70 °C
const MOTOR_TEMPERATURE_DANGER_THRESHOLD = KELVIN_OFFSET + 90; // 90 °C
const SWIVEL_TEMPERATURE_WARNING_THRESHOLD = KELVIN_OFFSET + 70; // 70 °C
const SWIVEL_TEMPERATURE_DANGER_THRESHOLD = KELVIN_OFFSET + 90; // 90 °C

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
    lastWheelTemperatures = msg;
    window.dispatchEvent(new Event('wheel-temps'));
  });
});

type ArrowProps = {
  velocity: number;
  angle: number;
  thickness: number;
};

function parseKelvinToCelsius(kelvin: number): string {
  const celsius = kelvin - KELVIN_OFFSET;
  return `${celsius.toFixed(2)}°C`;
}

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
  motorTemperature: number | null;
  swivelTemperature: number | null;
  showTarget: boolean;
};

function Wheel({ type, angle, velocity, motorTemperature, swivelTemperature, showTarget }: WheelProps) {
  const ref = useRef<HTMLDivElement>();
  const thickness = ref.current ? ref.current.clientWidth * 0.3 : 5;

  let additionalWheelClass: string = '';
  let additionalMotorTemperatureClass: string = '';
  let additionalSwivelTemperatureClass: string = '';

  if (
    motorTemperature >= MOTOR_TEMPERATURE_DANGER_THRESHOLD ||
    swivelTemperature >= SWIVEL_TEMPERATURE_DANGER_THRESHOLD
  ) {
    additionalWheelClass = ' wheel-danger';
  } else if (
    motorTemperature >= MOTOR_TEMPERATURE_WARNING_THRESHOLD ||
    swivelTemperature >= SWIVEL_TEMPERATURE_WARNING_THRESHOLD
  ) {
    additionalWheelClass = ' wheel-warn';
  }

  console.log(motorTemperature, additionalWheelClass);

  if (motorTemperature >= MOTOR_TEMPERATURE_DANGER_THRESHOLD) additionalWheelClass = ' danger';
  else if (motorTemperature >= MOTOR_TEMPERATURE_WARNING_THRESHOLD) additionalWheelClass = ' warn';

  if (swivelTemperature >= SWIVEL_TEMPERATURE_DANGER_THRESHOLD) additionalWheelClass = ' danger';
  else if (swivelTemperature >= SWIVEL_TEMPERATURE_WARNING_THRESHOLD) additionalWheelClass = ' warn';

  return (
    <>
      {!showTarget && lastWheelTemperatures !== null && (
        <div
          className={styles['wheel-temperatures'] + ' ' + styles[type]}
          style={{
            height: ref.current?.clientHeight
          }}
        >
          <p>
            Motor:{' '}
            <span className={styles['wheel-temperatures-measurement'] + additionalMotorTemperatureClass}>
              {parseKelvinToCelsius(motorTemperature)}
            </span>
          </p>
          <p>
            Swivel:{' '}
            <span className={styles['wheel-temperatures-measurement'] + additionalSwivelTemperatureClass}>
              {parseKelvinToCelsius(swivelTemperature)}
            </span>
          </p>
        </div>
      )}
      <div
        ref={ref}
        className={styles['wheel'] + ' ' + styles[type]}
        style={{
          transform: `rotate(${(-angle * 180) / Math.PI + (type === 'fr' || type === 'br' ? 180 : 0)}deg)`
        }}
      >
        <img
          src={showTarget ? kalmanLeftWheelOutline : kalmanLeftWheel}
          className={styles['wheel-image'] + additionalWheelClass}
          draggable='false'
          alt={''}
        />
        <Arrow velocity={velocity} angle={type === 'fr' || type === 'br' ? 180 : 0} thickness={thickness} />
      </div>
    </>
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
    window.addEventListener('resize', rerender);
    return () => {
      window.removeEventListener('wheel-states', rerender);
      window.removeEventListener('wheel-states-return', rerender);
      window.removeEventListener('wheel-temps', rerender);
      window.removeEventListener('resize', rerender);
    };
  }, [rerender]);

  const wheelStates = showTarget ? lastWheelStates : lastWheelStatesReturn;
  const wheelTemperatures = lastWheelTemperatures;
  return (
    <div className={styles['rover'] + (showTarget ? ' show-target' : '')}>
      <div className={styles['rover-h']}>
        <div className={styles['rover-v']}>
          {!showTarget && <img src={kalmanBody} className={styles['body']} draggable='false' alt={''} />}
          <Wheel
            type='fl'
            angle={wheelStates?.front_left.angle || 0}
            velocity={wheelStates?.front_left.velocity || 0}
            motorTemperature={wheelTemperatures?.front_left.motor}
            swivelTemperature={wheelTemperatures?.front_left.swivel}
            showTarget={showTarget}
          />
          <Wheel
            type='fr'
            angle={wheelStates?.front_right.angle || 0}
            velocity={wheelStates?.front_right.velocity || 0}
            motorTemperature={wheelTemperatures?.front_right.motor}
            swivelTemperature={wheelTemperatures?.front_right.swivel}
            showTarget={showTarget}
          />
          <Wheel
            type='bl'
            angle={wheelStates?.back_left.angle || 0}
            velocity={wheelStates?.back_left.velocity || 0}
            motorTemperature={wheelTemperatures?.back_left.motor}
            swivelTemperature={wheelTemperatures?.back_left.swivel}
            showTarget={showTarget}
          />
          <Wheel
            type='br'
            angle={wheelStates?.back_right.angle || 0}
            velocity={wheelStates?.back_right.velocity || 0}
            motorTemperature={wheelTemperatures?.back_right.motor}
            swivelTemperature={wheelTemperatures?.back_right.swivel}
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
