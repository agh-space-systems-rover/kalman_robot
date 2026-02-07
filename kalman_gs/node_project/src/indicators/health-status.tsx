import styles from './health-status.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { ReadPkgConfigFileRequest, ReadPkgConfigFileResponse, UInt8 } from '../common/ros-interfaces';
import { faCamera, faCar, faChartSimple, faCompass, faSatellite } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import yaml from 'js-yaml';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Service, Topic } from 'roslib';

import Tooltip from '../components/tooltip';

const INACTIVITY_TIMEOUT_SECONDS = 15;

type MonitorsConfig = {
  frequency: number;
  devices: DeviceConfig[];
};

type DeviceConfig = {
  name: string;
  topic: string;
  topic_type: string;
  timeout: number;
};

type Device = {
  id: number;
  name: string;
};

const DEVICE_ICONS: Record<string, any> = {
  IMU: faCompass,
  GPS: faSatellite,
  Master: faCar,
  'D455 Front': faCamera,
  'D455 Left': faCamera,
  'D455 Back': faCamera,
  'D455 Right': faCamera
};

let configReaderService: Service<ReadPkgConfigFileRequest, ReadPkgConfigFileResponse> | null = null;
let latestHealthStatus: UInt8 | null = null;

window.addEventListener('ros-connect', () => {
  configReaderService = new Service({
    ros: ros,
    name: '/read_pkg_config_file',
    serviceType: 'kalman_interfaces/ReadPkgConfigFile'
  });

  const healthStatusTopic = new Topic({
    ros: ros,
    name: '/topic_health_status',
    messageType: 'std_msgs/UInt8'
  });

  healthStatusTopic.subscribe((msg: UInt8) => {
    latestHealthStatus = msg;
    window.dispatchEvent(new CustomEvent('health-status-update'));
  });
});

export default function HealthStatus() {
  const [devices, setDevices] = useState<Device[] | null>(null);
  const [statuses, setStatuses] = useState<number[] | null>(null);
  const [isVisible, setIsVisible] = useState(false);
  const [hasError, setHasError] = useState(false);

  const hideTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const handleHealthStatusUpdate = useCallback(() => {
    if (!latestHealthStatus) return;

    const binaryString = latestHealthStatus.data.toString(2).padStart(8, '0');
    const statusArray = Array.from(binaryString).map((bit) => Number(bit));

    // Reset hide timeout
    if (hideTimeoutRef.current) {
      clearTimeout(hideTimeoutRef.current);
    }
    hideTimeoutRef.current = setTimeout(() => {
      setIsVisible(false);
    }, INACTIVITY_TIMEOUT_SECONDS * 1000);

    // Load device configuration on first update
    if (devices === null && configReaderService) {
      configReaderService.callService(
        { pkg: 'kalman_health', path: 'config/monitors.yaml' },
        (response: ReadPkgConfigFileResponse) => {
          if (response.success) {
            try {
              const config = yaml.load(response.content) as MonitorsConfig;
              const deviceList: Device[] = config.devices.map((device, index) => ({
                id: index,
                name: device.name
              }));
              setDevices(deviceList);
            } catch (error) {
              console.error('Failed to parse monitor config:', error);
              alertsRef.current?.pushAlert('Failed to parse health monitor config.', 'error');
            }
          } else {
            alertsRef.current?.pushAlert('Failed to read health monitor config.', 'error');
          }
        },
        (error) => {
          console.error('Config reader service error:', error);
          alertsRef.current?.pushAlert('Failed to call config reader service.', 'error');
        }
      );
    } else if (devices) {
      // Check if any device has an error
      const hasAnyError = devices.some((device) => statusArray[device.id] === 0);
      setHasError(hasAnyError);
    }

    setIsVisible(true);
    setStatuses(statusArray);
  }, [devices]);

  useEffect(() => {
    window.addEventListener('health-status-update', handleHealthStatusUpdate);
    return () => {
      window.removeEventListener('health-status-update', handleHealthStatusUpdate);
      if (hideTimeoutRef.current) {
        clearTimeout(hideTimeoutRef.current);
      }
    };
  }, [handleHealthStatusUpdate]);

  const renderTooltipContent = () => {
    if (!devices || !statuses) {
      return 'Loading device health status...';
    }

    return (
      <div className={styles['health-status-tooltip']}>
        <div className={styles['health-status-tooltip-header']}>Device Health Status</div>
        {devices.map((device) => (
          <div key={device.id} className={styles['health-status-tooltip-device']}>
            <FontAwesomeIcon
              icon={DEVICE_ICONS[device.name] || faChartSimple}
              className={styles['health-status-tooltip-icon'] + (statuses[device.id] ? ' connected' : ' disconnected')}
            />
            <span>{device.name}</span>
          </div>
        ))}
      </div>
    );
  };

  return (
    <Tooltip
      text={renderTooltipContent()}
      className={styles['health-status'] + (isVisible ? '' : ' no-display') + (hasError ? ' error' : ' connected')}
    >
      <FontAwesomeIcon icon={faChartSimple} style={{ pointerEvents: 'none' }} />
    </Tooltip>
  );
}
