import styles from './e-stop-button.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { faStopCircle, faCirclePlay } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

enum RoverState {
  ON = 0,
  OFF = 1
}

let eStopStopService: Service<{}, {}> = null;
let eStopStartService: Service<{}, {}> = null;
let warningNotify: any /* TODO: create type */ = null;
window.addEventListener('ros-connect', () => {
  eStopStopService = new Service<{}, {}>({
    ros: ros,
    name: '/e_stop/stop', // TODO: change name
    serviceType: 'std_srvs/Empty'
  });

  eStopStartService = new Service<{}, {}>({
    ros: ros,
    name: '/e_stop/start', // TODO: change name
    serviceType: 'std_srvs/Empty'
  });

  // Handle notification about potential threats
  const topic = new Topic({
    ros: ros,
    name: '/e_stop/warning', // TODO: change name
    messageType: '' // TODO: create/change messageType
  });

  topic.subscribe((msg: any /* TODO: create type */) => {
    warningNotify = msg;
    window.dispatchEvent(new Event('warning-notify'));
  });
});

export default function EStopButton() {
  // expectedRoverState is initially set to ON because we want the first request sent from the panel to be OFF
  // The opposite request can only be sent after receiving a confirmation
  const [expectedRoverState, setExpectedRoverState] = useState(RoverState.ON);
  const [warningExist, setWarningExist] = useState(false);

  const notifyWarning = useCallback(() => {
    // TODO: validate via warningNotify info
    console.log('xxx');
    if (expectedRoverState === RoverState.ON) {
      setWarningExist(true);
    }
  }, [setWarningExist]);

  useEffect(() => {
    window.addEventListener('warning-notify', notifyWarning);
    return () => {
      window.removeEventListener('warning-notify', notifyWarning);
    };
  }, [notifyWarning]);

  return (
    <Tooltip
      text='Show advanced settings.'
      className={styles['e-stop-button'] + (warningExist && expectedRoverState === RoverState.ON ? ' warning' : '')}
      onClick={() => {
        if (eStopStopService === null || eStopStartService === null) {
          alertsRef.current?.pushAlert('Not connected to local ROS instance. Unable to send e-stop request.', 'error');
          return;
        }

        if (expectedRoverState === RoverState.ON) {
          eStopStopService.callService(
            {},
            () => {
              alertsRef.current?.pushAlert(`Successfully disabled the rover's emergency power.`, 'success');
              setWarningExist(false);
              setExpectedRoverState(RoverState.OFF);
            },
            (e) => {
              alertsRef.current?.pushAlert(`Unable to send a reboot request. ${e}.`, 'error');
            }
          );
        } else {
          eStopStartService.callService(
            {},
            () => {
              alertsRef.current?.pushAlert(`Successfully enabled the rover's emergency power.`, 'success');
              setExpectedRoverState(RoverState.ON);
            },
            (e) => {
              alertsRef.current?.pushAlert(`Unable to send a reboot request. ${e}.`, 'error');
            }
          );
        }
      }}
    >
      <FontAwesomeIcon icon={expectedRoverState === RoverState.ON ? faStopCircle : faCirclePlay} />
    </Tooltip>
  );
}
