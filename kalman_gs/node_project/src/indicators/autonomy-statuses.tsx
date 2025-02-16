import styles from './autonomy-statuses.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { AutonomyStatus } from '../common/ros-interfaces';
import { faCamera, faCar, faCompass, faInfo, faSatellite } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

type Devices = {
  devices: Device[];
};

type Device = {
  id: number;
  device_name: string;
};

const deviceIcon: Record<number, any> = {
  0: faCompass, // IMU
  1: faSatellite, // GPS
  2: faCar, // Wheels
  3: faCamera, // d455_front
  4: faCamera, // d455_left
  5: faCamera, // d455_back
  6: faCamera // d455_right
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
  const [autonomyDeviceError, setAutonomyDeviceError] = useState(false);

  const updateAutonomyStatuses = useCallback(() => {
    if (autonomyStatusValue !== null) {
      const binaryString = autonomyStatusValue.status.toString(2).padStart(8, '0');
      const statusesArray = Array.from(binaryString).map((bit) => Number(bit));

      // Check for any inactive device, then set error
      if (autonomyDevices === null) {
        autonomyDevicesService.callService({}, (data: Devices) => setAutonomyDevices(data.devices), undefined);
      } else {
        let errorStatement = false;
        autonomyDevices.forEach((device: Device) => {
          if (autonomyStatuses[device.id] === 0) {
            errorStatement = true;
          }
        });
        setAutonomyDeviceError(errorStatement);
      }

      setAutonomyStatuses(statusesArray);
    }
  }, [autonomyDevices, autonomyStatuses, autonomyDeviceError]);

  useEffect(() => {
    window.addEventListener('autonomy-status-update', updateAutonomyStatuses);
    return () => {
      window.removeEventListener('autonomy-status-update', updateAutonomyStatuses);
    };
  }, [updateAutonomyStatuses]);

  const handleClickOutside = (event: MouseEvent) => {
    const targetElement = event.target as HTMLElement;

    if (targetElement.closest(`.${styles['autonomy-statuses']}`)) return;

    if (!targetElement.className.includes('autonomy-statuses-modal')) {
      setShowDevices(false);
    }
  };

  const handleKeyDown = (event: KeyboardEvent) => {
    if (event.key === 'Escape') {
      setShowDevices(false);
    }
  };

  useEffect(() => {
    if (showDevices) {
      document.addEventListener('mouseup', handleClickOutside);
      document.addEventListener('keydown', handleKeyDown);
    } else {
      document.removeEventListener('mouseup', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    }

    return () => {
      document.removeEventListener('mouseup', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [showDevices]);

  return (
    <>
      <Tooltip
        text='Show autonomy devices statuses.'
        className={
          styles['autonomy-statuses'] +
          (autonomyStatuses === null ? ' no-display' : '') +
          (autonomyDeviceError ? ' error' : '')
        }
        onClick={() => {
          if (autonomyStatuses === null) {
            alertsRef.current?.pushAlert(`Cannot get list of autonomy devices statuses.`, 'error');
            return;
          }

          setShowDevices((prev) => !prev);
        }}
      >
        <FontAwesomeIcon icon={faInfo} />
      </Tooltip>

      {autonomyDevices !== null && (
        <div className={styles['autonomy-statuses-modal'] + (showDevices ? ' shown' : '')}>
          <div className={styles['autonomy-statuses-modal-content']}>
            <h3 className={styles['autonomy-statuses-modal-header']}>Autonomy statuses</h3>
            {autonomyDevices.map((device: Device) => (
              <p className={styles['autonomy-statuses-modal-device']} key={device.id}>
                <FontAwesomeIcon
                  icon={deviceIcon[device.id]}
                  className={
                    styles['autonomy-statuses-modal-device-icon'] +
                    (autonomyStatuses[device.id] ? ' connected' : ' disconnected')
                  }
                />
                {device.device_name}
              </p>
            ))}
          </div>
        </div>
      )}
    </>
  );
}
