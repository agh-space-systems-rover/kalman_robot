import styles from './camera-2dof.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { Camera2Dof } from '../common/ros-interfaces';
import { faCamera, faGaugeHigh, faLeftRight, faUpDown } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

import Button from '../components/button';
import Checkbox from '../components/checkbox';
import Label from '../components/label';

let cameraTopic: Topic<Camera2Dof> | null = null;
window.addEventListener('ros-connect', () => {
  cameraTopic = new Topic<Camera2Dof>({
    ros,
    name: '/camera_2dof/cmd',
    messageType: 'kalman_interfaces/Camera2Dof'
  });
});

type AxisSliderProps = {
  label: string;
  labelColor: string;
  labelTooltip: string;
  icon: any;
  value: number;
  min: number;
  max: number;
  unit?: string;
  onChange: (value: number) => void;
};

function AxisSlider({ label, labelColor, labelTooltip, icon, value, min, max, unit = '°', onChange }: AxisSliderProps) {
  return (
    <div className={styles['axis-row']}>
      <Label color={labelColor} tooltip={labelTooltip}>
        <FontAwesomeIcon icon={icon} />
        <span className={styles['axis-label-text']}>{label}</span>
      </Label>
      <div className={styles['slider-container']}>
        <input
          className={styles['slider']}
          type="range"
          min={min}
          max={max}
          step={1}
          value={value}
          onChange={(e) => onChange(Number(e.target.value))}
        />
        <div className={styles['slider-labels']}>
          <span>{min}{unit}</span>
          <span className={styles['slider-value']}>{value}{unit}</span>
          <span>{max}{unit}</span>
        </div>
      </div>
    </div>
  );
}

export default function Camera2DofPanel() {
  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const greenBg = style.getPropertyValue('--green-background');
  const blueBg = style.getPropertyValue('--blue-background');
  const yellowBg = style.getPropertyValue('--yellow-background');

  const [cameraId, setCameraId] = useState(0);
  const [yaw, setYaw] = useState(90);
  const [pitch, setPitch] = useState(90);
  const [autoSend, setAutoSend] = useState(false);
  const [rate, setRate] = useState(10);

  // Keep refs so the interval always reads latest values without needing restart.
  const yawRef = useRef(yaw);
  const pitchRef = useRef(pitch);
  const cameraIdRef = useRef(cameraId);
  yawRef.current = yaw;
  pitchRef.current = pitch;
  cameraIdRef.current = cameraId;

  function publish(id: number, y: number, p: number) {
    if (!cameraTopic) {
      alertsRef.current?.pushAlert('Camera 2DOF: ROS not connected.', 'error');
      return;
    }
    cameraTopic.publish({ camera_id: id, yaw: y, pitch: p });
  }

  // Start / stop the publish interval when autoSend or rate changes.
  useEffect(() => {
    if (!autoSend) return;

    const id = setInterval(() => {
      publish(cameraIdRef.current, yawRef.current, pitchRef.current);
    }, 1000 / rate);

    return () => clearInterval(id);
  }, [autoSend, rate]);

  return (
    <div className={styles['camera-2dof']}>
      <div className={styles['camera-id-row']}>
        <Label color={blueBg} tooltip="Camera ID">
          <FontAwesomeIcon icon={faCamera} />
        </Label>
        <Button tooltip="Previous camera" onClick={() => setCameraId(Math.max(0, cameraId - 1))}>
          −
        </Button>
        <span className={styles['camera-id-value']}>{cameraId}</span>
        <Button tooltip="Next camera" onClick={() => setCameraId(Math.min(7, cameraId + 1))}>
          +
        </Button>
      </div>

      <AxisSlider
        label="YAW"
        labelColor={redBg}
        labelTooltip="Yaw — left/right"
        icon={faLeftRight}
        value={yaw}
        min={0}
        max={180}
        onChange={setYaw}
      />

      <AxisSlider
        label="PCH"
        labelColor={greenBg}
        labelTooltip="Pitch — up/down"
        icon={faUpDown}
        value={pitch}
        min={0}
        max={180}
        onChange={setPitch}
      />

      <div className={styles['divider']} />

      <div className={styles['bottom-row']}>
        <Checkbox checked={autoSend} onChange={setAutoSend} label="Auto-send" />
      </div>

      {autoSend ? (
        <AxisSlider
          label="HZ"
          labelColor={yellowBg}
          labelTooltip="Publish rate (per second)"
          icon={faGaugeHigh}
          value={rate}
          min={1}
          max={60}
          unit="/s"
          onChange={setRate}
        />
      ) : (
        <div className={styles['bottom-row']}>
          <Button
            className={styles['send-button']}
            tooltip="Publish camera position to ROS"
            onClick={() => publish(cameraId, yaw, pitch)}
          >
            Send
          </Button>
        </div>
      )}
    </div>
  );
}
