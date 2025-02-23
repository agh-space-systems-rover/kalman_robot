import styles from './topic-health-monitor.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { UInt8 } from '../common/ros-interfaces';
import { faCamera, faCar, faCompass, faInfo, faSatellite } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

const TOPIC_INACTIVITY_SECONDS = 15;

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
  2: faCar, // Master
  3: faCamera, // d455_front
  4: faCamera, // d455_left
  5: faCamera, // d455_back
  6: faCamera // d455_right
};

// Services:
let healthDevicesService: Service<{}, Devices> = null;

// Topic variables:
let healthStatusValue: UInt8 | null = null;

window.addEventListener('ros-connect', () => {
  healthDevicesService = new Service({
    ros: ros,
    name: '/topic_health_status/get_devices',
    serviceType: 'kalman_interfaces/GetDevices'
  });

  const healthTopic = new Topic({
    ros: ros,
    name: '/topic_health_status',
    messageType: 'std_msgs/UInt8'
  });

  healthTopic.subscribe((msg: UInt8) => {
    healthStatusValue = msg;
    window.dispatchEvent(new Event('topic-health-monitor-update'));
  });
});

export default function TopicHealthMonitors() {
  const [healthDevices, setHealthDevices] = useState(null);
  const [healthStatuses, setHealthStatuses] = useState(null);
  const [showIcon, setShowIcon] = useState(false);
  const [showDevices, setShowDevices] = useState(false);
  const [healthDeviceError, setHealthDeviceError] = useState(false);

  const timerRef = useRef<NodeJS.Timeout | null>(null);

  const updateTopicHealthMonitors = useCallback(() => {
    if (healthStatusValue !== null) {
      const binaryString = healthStatusValue.data.toString(2).padStart(8, '0');
      const statusesArray = Array.from(binaryString).map((bit) => Number(bit));

      // Set timeout to hide the info panel after 20 seconds of inactivity
      if (timerRef.current) {
        clearTimeout(timerRef.current);
      }
      timerRef.current = setTimeout(() => {
        setShowIcon(false);
      }, TOPIC_INACTIVITY_SECONDS * 1000);

      // Check for any inactive device, then set error
      if (healthDevices === null) {
        healthDevicesService.callService({}, (data: Devices) => setHealthDevices(data.devices), undefined);
      } else {
        let errorStatement = false;
        healthDevices.forEach((device: Device) => {
          if (healthStatuses[device.id] === 0) {
            errorStatement = true;
          }
        });
        setHealthDeviceError(errorStatement);
      }

      setShowIcon(true);
      setHealthStatuses(statusesArray);
    }
  }, [healthDevices, healthStatuses, healthDeviceError]);

  useEffect(() => {
    window.addEventListener('topic-health-monitor-update', updateTopicHealthMonitors);
    return () => {
      window.removeEventListener('topic-health-monitor-update', updateTopicHealthMonitors);
    };
  }, [updateTopicHealthMonitors]);

  const handleClickOutside = (event: MouseEvent) => {
    const targetElement = event.target as HTMLElement;

    if (targetElement.closest(`.${styles['topic-health-monitors']}`)) return;

    if (!targetElement.className.includes('topic-health-monitors-modal')) {
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
        text='Show devices healthchecks.'
        className={
          styles['topic-health-monitors'] + (showIcon ? '' : ' no-display') + (healthDeviceError ? ' error' : '')
        }
        onClick={() => {
          if (healthStatuses === null) {
            alertsRef.current?.pushAlert(`Cannot get list of devices statuses.`, 'error');
            return;
          }

          setShowDevices((prev) => !prev);
        }}
      >
        <FontAwesomeIcon icon={faInfo} />
      </Tooltip>

      {healthDevices !== null && (
        <div className={styles['topic-health-monitors-modal'] + (showDevices ? ' shown' : '')}>
          <div className={styles['topic-health-monitors-modal-content']}>
            <h3 className={styles['topic-health-monitors-modal-header']}>Health statuses</h3>
            {healthDevices.map((device: Device) => (
              <p className={styles['topic-health-monitors-modal-device']} key={device.id}>
                <FontAwesomeIcon
                  icon={deviceIcon[device.id]}
                  className={
                    styles['topic-health-monitors-modal-device-icon'] +
                    (healthStatuses[device.id] ? ' connected' : ' disconnected')
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
