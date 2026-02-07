import styles from './mobile.module.css';

import { ros } from '../common/ros';
import { Drive } from '../common/ros-interfaces';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

const RATE = 30;
const SPEED = 1.0;
const TURN_RADIUS = 0.75;
const ROTATE_IN_PLACE_SPEED = 1.57;

let lastDrive: Drive = null;

let xy = { x: 0, y: 0 };
let rotation = 0;

function clamp(x: number, lower: number, upper: number) {
  return Math.max(lower, Math.min(x, upper));
}

window.addEventListener('ros-connect', () => {
  const drive = new Topic<Drive>({
    ros: ros,
    name: '/drive',
    messageType: 'kalman_interfaces/Drive'
  });

  setInterval(() => {
    const msg: Drive = {
      speed: -xy.y * SPEED,
      inv_radius: -xy.x / TURN_RADIUS,
      rotation: -rotation * ROTATE_IN_PLACE_SPEED
    };

    if (msg.speed === 0 && msg.inv_radius === 0 && msg.rotation === 0) {
      if (JSON.stringify(lastDrive) === JSON.stringify(msg)) {
        return;
      }
    }

    lastDrive = msg;
    drive.publish(msg);
  }, 1000 / RATE);
});

// This panel has a joystick that can be tilted in two axes to control the robot's movement.
// It is intended to be used on a mobile device to control the robot.
export default function Mobile() {
  const holdingControl = useRef(0); // 0 = not holding, 1 = holding joystick, 2 = holding slider
  const mouseCatcherRef = useRef<HTMLDivElement | null>(null);
  const joystickRef = useRef<HTMLDivElement | null>(null);
  const handleRef = useRef<HTMLDivElement | null>(null);
  const sliderRef = useRef<HTMLDivElement | null>(null);
  const sliderHandleRef = useRef<HTMLDivElement | null>(null);

  const setHoldingControl = useCallback((value: number) => {
    holdingControl.current = value;
    if (holdingControl.current !== 0) {
      // set display
      mouseCatcherRef.current?.style.setProperty('display', 'block');
    } else {
      // set display
      mouseCatcherRef.current?.style.setProperty('display', 'none');
      xy = { x: 0, y: 0 };
      rotation = 0;
      const handle = handleRef.current;
      const sliderHandle = sliderHandleRef.current;
      if (!handle || !sliderHandle) return;
      handle.style.left = '50%';
      handle.style.top = '50%';
      sliderHandle.style.left = '50%';
    }
  }, []);

  const updateControl = useCallback((event: any) => {
    if (holdingControl.current === 0) return;

    if (holdingControl.current === 1) {
      const joystick = joystickRef.current;
      const handle = handleRef.current;
      if (!joystick || !handle) return;

      let x = (event.clientX - joystick.getBoundingClientRect().left) / joystick.clientWidth;
      let y = (event.clientY - joystick.getBoundingClientRect().top) / joystick.clientHeight;

      x = x * 2 - 1;
      y = y * 2 - 1;

      // Limit the length of x,y
      const length = Math.sqrt(x * x + y * y);
      x = (x / length) * Math.min(length, 0.7);
      y = (y / length) * Math.min(length, 0.7);

      handle.style.left = `${(x * 0.5 + 0.5) * 100}%`;
      handle.style.top = `${(y * 0.5 + 0.5) * 100}%`;

      x = clamp((x / 0.7) * 1.3, -1, 1);
      y = clamp((y / 0.7) * 1.3, -1, 1);

      xy = { x, y };
    } else if (holdingControl.current === 2) {
      const slider = sliderRef.current;
      const sliderHandle = sliderHandleRef.current;
      if (!slider || !sliderHandle) return;

      let x = (event.clientX - slider.getBoundingClientRect().left) / slider.clientWidth;

      x = x * 2 - 1;

      // Limit the length of x
      const length = Math.abs(x);
      x = (x / length) * Math.min(length, 0.85);

      sliderHandle.style.left = `${(x * 0.5 + 0.5) * 100}%`;

      x = clamp((x / 0.85) * 1.3, -1, 1);

      rotation = x;
    }
  }, []);

  return (
    <div className={styles['mobile']}>
      <div
        className={styles['mouse-catcher']}
        ref={mouseCatcherRef}
        onMouseUp={() => {
          setHoldingControl(0);
        }}
        onTouchEnd={() => {
          setHoldingControl(0);
        }}
        onMouseMove={updateControl}
        onTouchMove={(event) => {
          updateControl(event.touches[0]);
        }}
      />
      {/* <div className={styles['joystick-h']}>
        <div className={styles['joystick-v']}> */}
      <div
        className={styles['joystick']}
        onMouseDown={(event) => {
          setHoldingControl(1);
          updateControl(event);
        }}
        onTouchStart={(event) => {
          setHoldingControl(1);
          updateControl(event.touches[0]);
        }}
        onMouseUp={() => {
          setHoldingControl(0);
        }}
        onTouchEnd={() => {
          setHoldingControl(0);
        }}
        onMouseMove={updateControl}
        onTouchMove={(event) => {
          updateControl(event.touches[0]);
        }}
        ref={joystickRef}
      >
        <div className={styles['handle']} ref={handleRef} />
      </div>
      <div
        className={styles['slider']}
        onMouseDown={(event) => {
          setHoldingControl(2);
          updateControl(event);
        }}
        onTouchStart={(event) => {
          setHoldingControl(2);
          updateControl(event.touches[0]);
        }}
        onMouseUp={() => {
          setHoldingControl(0);
        }}
        onTouchEnd={() => {
          setHoldingControl(0);
        }}
        onMouseMove={updateControl}
        onTouchMove={(event) => {
          updateControl(event.touches[0]);
        }}
        ref={sliderRef}
      >
        <div className={styles['slider-handle']} ref={sliderHandleRef} />
      </div>
      {/* </div>
      </div> */}
    </div>
  );
}
