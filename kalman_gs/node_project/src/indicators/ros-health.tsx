import styles from './ros-health.module.css';

import { ros } from '../common/ros';
import { faRobot } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';

import Tooltip from '../components/tooltip';

export default function RosHealth() {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const interval = setInterval(() => {
      setConnected(ros.isConnected);
    }, 500);
    return () => {
      clearInterval(interval);
    };
  }, [setConnected]);

  return (
    <Tooltip
      key={connected ? 1 : 0}
      text={
        connected
          ? 'ROS is connected.'
          : 'ROS network is not available. Please wait for connection.'
      }
      className={
        styles['ros-health'] + (connected ? ' connected' : ' disconnected')
      }
    >
      <FontAwesomeIcon icon={faRobot} />
    </Tooltip>
  );
}
