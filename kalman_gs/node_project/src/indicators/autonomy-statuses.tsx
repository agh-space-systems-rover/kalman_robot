import styles from './autonomy-statuses.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { AutonomyStatus } from '../common/ros-interfaces';
import { faInfo } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

type Devices = {
  devices: Device[];
};

type Device = {
  id: number;
  display_name: string;
};

// Services:
let autonomyDevicesService: Service<{}, Devices> = null;

// Topic variables:
let autonomyStatusValue: AutonomyStatus | null = null;

window.addEventListener('ros-connect', () => {
  autonomyDevicesService = new Service({
    ros: ros,
    name: '/autonomy/get_devices',
    serviceType: 'kalman_interfaces/GetDevices'
  });

  const autonomyTopic = new Topic({
    ros: ros,
    name: '/autonomy_status',
    messageType: 'kalman_interfaces/AutonomyStatus'
  });

  autonomyTopic.subscribe((msg: AutonomyStatus) => {
    autonomyStatusValue = msg;
    window.dispatchEvent(new Event('autonomy-status-update'));
  });
});

export default function AutonomyStatuses() {
  const [autonomyDevices, setAutonomyDevices] = useState(null);
  const [autonomyStatuses, setAutonomyStatuses] = useState(null);
  const [showDevices, setShowDevices] = useState(false);

  const updateAutonomyStatuses = useCallback(() => {
    if (autonomyStatusValue !== null) {
      const binaryString = autonomyStatusValue.status.toString(2).padStart(8, '0');
      const statusesArray = Array.from(binaryString).map((bit) => Number(bit));

      setAutonomyStatuses(statusesArray);
    }
  }, []);

  useEffect(() => {
    window.addEventListener('autonomy-status-update', updateAutonomyStatuses);
    return () => {
      window.removeEventListener('autonomy-status-update', updateAutonomyStatuses);
    };
  }, [updateAutonomyStatuses]);

  return (
    <>
      <Tooltip
        text='Show autonomy componenets statuses.'
        className={styles['autonomy-statuses']}
        onClick={() => {
          if (autonomyStatuses === null) {
            alertsRef.current?.pushAlert(`Cannot get list of autonomy devices statuses.`, 'error');
            return;
          }

          if (autonomyDevices === null) {
            autonomyDevicesService.callService(
              {},
              (data: Devices) => setAutonomyDevices(data.devices),
              (e) => alertsRef.current?.pushAlert(`Cannot get list of autonomy devices. ${e}.`, 'error')
            );
          }

          setShowDevices(true);
        }}
      >
        <FontAwesomeIcon icon={faInfo} />
      </Tooltip>
    </>
  );
}
