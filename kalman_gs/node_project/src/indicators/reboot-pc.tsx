import styles from './reboot-pc.module.css';

import { faPowerOff } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Service } from 'roslib';

import Button from '../components/button';
import Tooltip from '../components/tooltip';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';

let rebootService: Service<{}, {}> = null;
window.addEventListener('ros-connect', () => {
  rebootService = new Service({
    ros: ros,
    name: '/tunnel/reboot_pc',
    serviceType: 'std_srvs/Empty'
  });
});

export default function RebootPC() {
  return (
    <Tooltip
      text='Reboot the onboard computer.'
      className={styles['reboot-pc']}
      onClick={() => {
        if (rebootService === null) {
          alertsRef.current?.pushAlert(
            'Not connected to local ROS instance. Unable to send reboot request.',
            'error'
          );
          return;
        }

        rebootService.callService(
          {},
          () => {
            alertsRef.current?.pushAlert(
              'A reboot request has been sent. It is a best effort operation. Multiple clicks may be necessary.',
              'success'
            );
          },
          (e) => {
            alertsRef.current?.pushAlert(
              `Unable to send a reboot request. ${e}.`,
              'error'
            );
          }
        );
      }}
    >
      <FontAwesomeIcon icon={faPowerOff} />
    </Tooltip>
  );
}
