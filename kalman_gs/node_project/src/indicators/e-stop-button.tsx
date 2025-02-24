import styles from './e-stop-button.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { Bool, SetBool, SetBoolFeedback } from '../common/ros-interfaces';
import { faStopCircle, faCirclePlay } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

const ICON_DISABLED_TIMEOUT = 500;

enum EStopStatus {
  ON = 1,
  OFF = 0
}

// Services:
let eStopService: Service<SetBool, SetBoolFeedback> = null;

// Topic variables:
let eStopTopicResponse: Bool | null = null;

window.addEventListener('ros-connect', () => {
  eStopService = new Service<SetBool, SetBoolFeedback>({
    ros: ros,
    name: '/set_estop',
    serviceType: 'std_srvs/SetBool'
  });

  const eStopTopic = new Topic({
    ros: ros,
    name: '/estop_state',
    messageType: 'std_msgs/Bool'
  });

  eStopTopic.subscribe((msg: Bool) => {
    eStopTopicResponse = msg;
    window.dispatchEvent(new Event('e-stop-status'));
  });
});

export default function EStopButton() {
  const [eStopStatus, setEStopStatus] = useState(EStopStatus.OFF);
  const [iconDisabled, setIconDisabled] = useState(true);

  const changeEStopStatus = useCallback(() => {
    if (eStopTopicResponse !== null && (eStopTopicResponse.data ? EStopStatus.ON : EStopStatus.OFF) !== eStopStatus) {
      setEStopStatus(eStopTopicResponse.data ? EStopStatus.ON : EStopStatus.OFF);
      setIconDisabled(true);
      setTimeout(() => {
        setIconDisabled(false);
      }, ICON_DISABLED_TIMEOUT);
    }
  }, [eStopStatus, iconDisabled]);

  useEffect(() => {
    window.addEventListener('e-stop-status', changeEStopStatus);
    return () => {
      window.removeEventListener('e-stop-status', changeEStopStatus);
    };
  }, [changeEStopStatus]);

  return (
    <Tooltip
      text={
        iconDisabled
          ? 'Unable to request.\\nE-STOP state is unknown.'
          : `${eStopStatus ? 'Enable' : 'Disable'} rover's power.`
      }
      className={styles['e-stop-button'] + (eStopStatus ? ' green' : ' red') + (iconDisabled ? ' disabled' : '')}
      onClick={() => {
        if (iconDisabled) {
          return;
        }

        if (eStopService === null) {
          alertsRef.current?.pushAlert('Not connected to local ROS instance. Unable to send E-STOP request.', 'error');
          return;
        }

        const setBoolRequest: SetBool = { data: eStopStatus === EStopStatus.OFF };
        eStopService.callService(
          setBoolRequest,
          (response: SetBoolFeedback) => {
            if (response.success) {
              alertsRef.current?.pushAlert(`Successfully updated E-STOP status.`, 'success');
            } else {
              alertsRef.current?.pushAlert(`Failed to update E-STOP status. ${response.message}.`, 'error');
            }
          },
          (e) => {
            alertsRef.current?.pushAlert(`Failed to update E-STOP status. ${e}.`, 'error');
          }
        );
      }}
    >
      <FontAwesomeIcon icon={eStopStatus === EStopStatus.ON ? faCirclePlay : faStopCircle} />
    </Tooltip>
  );
}
