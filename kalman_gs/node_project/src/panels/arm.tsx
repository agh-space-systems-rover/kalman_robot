import styles from './arm.module.css';

import { ros } from '../common/ros';
import { JointState } from '../common/ros-interfaces';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

let lastJointState: JointState | null = null;
window.addEventListener('ros-connect', () => {
  const topic = new Topic({
    ros: ros,
    name: '/arm_controllers/joint_states',
    messageType: 'sensor_msgs/JointState'
  });

  topic.subscribe((msg: JointState) => {
    lastJointState = msg;
    window.dispatchEvent(new Event('joint-state'));
  });
});

const jointLimitsRad = [
  { min: -6.2815926, max: 6.2815926 },
  { min: -3.1415926, max: 3.1415926 },
  { min: -2.88, max: 2.88 },
  { min: -6.4, max: 6.4 },
  { min: -1.75, max: 1.75 },
  { min: -3.4032, max: 3.4032 }
];

function rad2deg(rad: number) {
  return (rad * 180) / Math.PI;
}
function ArmStatus() {
  const [_, setRerenderCount] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, [setRerenderCount]);

  useEffect(() => {
    window.addEventListener('joint-state', rerender);
    return () => {
      window.removeEventListener('joint-state', rerender);
    };
  }, [rerender]);

  let namesAndValues = lastJointState
    ? lastJointState.name.map((name, i) => {
        return { name: name, value: lastJointState.position[i] };
      })
    : [];

  namesAndValues.sort((a, b) => {
    return a.name.localeCompare(b.name);
  });

  const jointNames = Array.from({ length: 6 }, (_, i) => (
    <div className={styles['joint-name']} key={i}>
      Joint {i + 1}:
    </div>
  ));

  const jointRangeLeft = jointLimitsRad.map((limit, i) => (
    <div className={styles['joint-range']} key={i}>
      {rad2deg(limit.min).toFixed(0)}..
    </div>
  ));

  const WARN_THRESHOLD = (20 * Math.PI) / 180;
  const ERROR_THRESHOLD = (5 * Math.PI) / 180;

  const distancesFromLimits = namesAndValues.map((joint, i) => {
    return Math.min(
      joint.value - jointLimitsRad[i].min,
      jointLimitsRad[i].max - joint.value
    );
  });

  const stylesFromLimits = distancesFromLimits.map((distance) => {
    if (distance < ERROR_THRESHOLD) {
      return styles['error'] + ' ';
    }
    if (distance < WARN_THRESHOLD) {
      return styles['warn'] + ' ';
    }
    return '';
  });

  const jointValues = Array.from({ length: 6 }, (_, i) => (
    <div
      className={
        (stylesFromLimits[i] ? stylesFromLimits[i] : styles['error'] + ' ') +
        styles['joint-value']
      }
      key={i}
    >
      {lastJointState ? rad2deg(namesAndValues[i].value).toFixed(0) : 'N/A'}
    </div>
  ));

  const jointRangeRight = jointLimitsRad.map((limit, i) => (
    <div className={styles['joint-range']} key={i}>
      ..{rad2deg(limit.max).toFixed(0)}
    </div>
  ));
  return (
    <div className={styles['arm-status']}>
      <h1 className={styles['status-header']}>Arm Status</h1>
      <div className={styles['status']}>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>
          {jointNames}
        </div>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>
          {jointRangeLeft}
        </div>
        <div className={styles['joint-column']}>{jointValues}</div>
        <div className={styles['joint-column'] + ' ' + styles['align-right']}>
          {jointRangeRight}
        </div>
      </div>
    </div>
  );
}

export default function Arms() {
  return (
    <div className={styles['arm-panel']}>
      <ArmStatus />
    </div>
  );
}
