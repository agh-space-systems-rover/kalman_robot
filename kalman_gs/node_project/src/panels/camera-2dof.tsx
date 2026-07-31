import styles from './camera-2dof.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { Camera2Dof } from '../common/ros-interfaces';
import { faCrosshairs, faGaugeHigh } from '@fortawesome/free-solid-svg-icons';
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

export default function Camera2DofPanel() {
  const style = getComputedStyle(document.body);
  const yellowBg = style.getPropertyValue('--yellow-background');

  // pos: { x, y } in [0, 1] — (0,0) = top-left, (1,1) = bottom-right
  const [pos, setPos] = useState({ x: 0.5, y: 0.5 });
  const [autoSend, setAutoSend] = useState(false);
  const [rate, setRate] = useState(10);

  // Ref so the interval always reads the latest pos without restart.
  const posRef = useRef(pos);
  posRef.current = pos;

  const containerRef = useRef<HTMLDivElement>(null);
  const isDragging = useRef(false);

  const yawDisplay = (pos.x * 180 - 90).toFixed(1);
  const pitchDisplay = (pos.y * 180 - 90).toFixed(1);

  function publish(x: number, y: number) {
    if (!cameraTopic) {
      alertsRef.current?.pushAlert('Camera 2DOF: ROS not connected.', 'error');
      return;
    }
    cameraTopic.publish({ camera_id: 0, yaw: Math.round(x * 180), pitch: Math.round(y * 180) });
  }

  useEffect(() => {
    if (!autoSend) return;
    const id = setInterval(() => {
      const p = posRef.current;
      publish(p.x, p.y);
    }, 1000 / rate);
    return () => clearInterval(id);
  }, [autoSend, rate]);

  function updateFromPointer(e: React.PointerEvent) {
    const container = containerRef.current;
    if (!container) return;
    const rect = container.getBoundingClientRect();
    const x = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
    const y = Math.max(0, Math.min(1, (e.clientY - rect.top) / rect.height));
    setPos({ x, y });
  }

  function handlePointerDown(e: React.PointerEvent) {
    isDragging.current = true;
    containerRef.current?.setPointerCapture(e.pointerId);
    updateFromPointer(e);
  }

  function handlePointerMove(e: React.PointerEvent) {
    if (!isDragging.current) return;
    updateFromPointer(e);
  }

  function handlePointerUp() {
    isDragging.current = false;
  }

  return (
    <div className={styles['camera-2dof']}>
      <div className={styles['joystick-wrapper']}>
        <div
          className={styles['joystick-area']}
          ref={containerRef}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
        >
          <div className={styles['joystick-crosshair-h']} style={{ top: `${pos.y * 100}%` }} />
          <div className={styles['joystick-crosshair-v']} style={{ left: `${pos.x * 100}%` }} />
          <div className={styles['joystick-thumb']} style={{ left: `${pos.x * 100}%`, top: `${pos.y * 100}%` }} />
        </div>
      </div>

      <div className={styles['values-row']}>
        <span className={styles['value-entry']}>
          <span className={styles['value-label']}>YAW</span>
          <span className={styles['value-number']}>{yawDisplay}°</span>
        </span>
        <span className={styles['value-entry']}>
          <span className={styles['value-label']}>PCH</span>
          <span className={styles['value-number']}>{pitchDisplay}°</span>
        </span>
      </div>

      <div className={styles['divider']} />

      <div className={styles['controls-row']}>
        <Checkbox checked={autoSend} onChange={setAutoSend} label='Auto-send' />
        <Button tooltip='Center joystick (90° / 90°)' onClick={() => setPos({ x: 0.5, y: 0.5 })}>
          <FontAwesomeIcon icon={faCrosshairs} />
        </Button>
      </div>

      {autoSend ? (
        <div className={styles['rate-row']}>
          <Label color={yellowBg} tooltip='Publish rate (Hz)'>
            <FontAwesomeIcon icon={faGaugeHigh} />
            <span className={styles['rate-label-text']}>HZ</span>
          </Label>
          <div className={styles['slider-container']}>
            <input
              className={styles['slider']}
              type='range'
              min={1}
              max={60}
              step={1}
              value={rate}
              onChange={(e) => setRate(Number(e.target.value))}
            />
            <div className={styles['slider-labels']}>
              <span>1/s</span>
              <span className={styles['slider-value']}>{rate}/s</span>
              <span>60/s</span>
            </div>
          </div>
        </div>
      ) : (
        <Button
          className={styles['send-button']}
          tooltip='Publish camera position to ROS'
          onClick={() => publish(pos.x, pos.y)}
        >
          Send
        </Button>
      )}
    </div>
  );
}
