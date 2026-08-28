import styles from './drill.module.css';

import { ros } from '../common/ros';
import { DrillTelemetry } from '../common/ros-interfaces';
import {
  faArrowDown,
  faArrowLeft,
  faArrowRight,
  faArrowRotateRight,
  faArrowUp,
  faHouse,
  faMinus,
  faPaperPlane,
  faPlay,
  faPlus,
  faStop,
  faWeightHanging
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

const RACK_MIN = -20;
const RACK_MAX = 20;
const DRILL_MIN = -100;
const DRILL_MAX = 100;
const SERVO_MIN_ANGLE = 0;
const SERVO_MAX_ANGLE = 270;
const SERVO_CHANNELS = [0, 3, 5, 10, 12, 15] as const;
type ServoChannel = (typeof SERVO_CHANNELS)[number];

const INITIAL_SERVO_ANGLES: Record<ServoChannel, number> = {
  0: 0,
  3: 0,
  5: 0,
  10: 0,
  12: 0,
  15: 0
};

let rackTopic: Topic<{ data: number }> | undefined;
let drillTopic: Topic<{ data: number }> | undefined;
let servoTopic: Topic<{ data: number[] }> | undefined;
let weightCmdTopic: Topic<{ data: number }> | undefined;
let autonomyTopic: Topic<{ data: number }> | undefined;
let drillTelemetryTopic: Topic<DrillTelemetry> | undefined;
let drillWeightTopic: Topic<{ data: number }> | undefined;

enum AutonomyState {
  Stop = 0,
  Start = 1,
  AutonomyStop = 2,
  Home = 3
}

type DrillGamepadUpdate = {
  rack?: number;
  drill?: number;
  autonomy?: AutonomyState;
  weightCommand?: 0 | 1;
};

const ensureDrillTopics = () => {
  if (!ros.isConnected) return;

  rackTopic ??= new Topic({
    ros,
    name: '/science/drill/rack',
    messageType: 'std_msgs/Int8'
  });
  drillTopic ??= new Topic({
    ros,
    name: '/science/drill/drill',
    messageType: 'std_msgs/Int8'
  });
  servoTopic ??= new Topic({
    ros,
    name: '/science/drill/servo',
    messageType: 'std_msgs/UInt16MultiArray'
  });
  weightCmdTopic ??= new Topic({
    ros,
    name: '/science/drill/weight/cmd',
    messageType: 'std_msgs/UInt8'
  });
  autonomyTopic ??= new Topic({
    ros,
    name: '/science/drill/autonomy',
    messageType: 'std_msgs/UInt8'
  });
  drillTelemetryTopic ??= new Topic({
    ros,
    name: '/science/drill/telemetry',
    messageType: 'kalman_interfaces/DrillTelemetry'
  });
  drillWeightTopic ??= new Topic({
    ros,
    name: '/science/drill/weight',
    messageType: 'std_msgs/Float32'
  });
};

window.addEventListener('ros-connect', () => {
  ensureDrillTopics();
  window.dispatchEvent(new CustomEvent('drill-subscribed'));
});

const normalizeInteger = (value: unknown, min: number, max: number, fallback: number) => {
  const parsedValue = typeof value === 'number' ? value : parseFloat(String(value));
  if (!Number.isFinite(parsedValue)) return fallback;
  return Math.max(min, Math.min(Math.round(parsedValue), max));
};

export default function Drill() {
  const rackInputRef = useRef<Input>(null);
  const drillInputRef = useRef<Input>(null);
  const servoInputRefs = useRef<Partial<Record<ServoChannel, Input | null>>>({});
  const rackSkipBlurRef = useRef(false);
  const drillSkipBlurRef = useRef(false);
  const servoSkipBlurRefs = useRef<Partial<Record<ServoChannel, boolean>>>({});

  const [rackValue, setRackValue] = useState(0);
  const [drillValue, setDrillValue] = useState(0);
  const [servoAngles, setServoAngles] = useState(INITIAL_SERVO_ANGLES);
  const [selectedAutonomy, setSelectedAutonomy] = useState<AutonomyState | null>(null);
  const [activeWeightCommand, setActiveWeightCommand] = useState<0 | 1 | null>(null);
  const [telemetry, setTelemetry] = useState<DrillTelemetry | null>(null);
  const [weight, setWeight] = useState<number | null>(null);
  const [lastTelemetryAt, setLastTelemetryAt] = useState<number | null>(null);
  const [lastWeightAt, setLastWeightAt] = useState<number | null>(null);
  const [now, setNow] = useState(Date.now());
  const [rerenderCount, setRerenderCount] = useState(0);

  useEffect(() => {
    const updateDrill = () => setRerenderCount((count) => count + 1);
    const updateFromGamepad = (event: Event) => {
      const detail = (event as CustomEvent<DrillGamepadUpdate>).detail;
      if (detail.rack !== undefined) {
        setRackValue(detail.rack);
        rackInputRef.current?.setValue(detail.rack);
      }
      if (detail.drill !== undefined) {
        setDrillValue(detail.drill);
        drillInputRef.current?.setValue(detail.drill);
      }
      if (detail.autonomy !== undefined) setSelectedAutonomy(detail.autonomy);
      if (detail.weightCommand !== undefined) {
        const command = detail.weightCommand;
        setActiveWeightCommand(command);
        window.setTimeout(() => setActiveWeightCommand((active) => (active === command ? null : active)), 200);
      }
    };

    ensureDrillTopics();
    window.addEventListener('drill-subscribed', updateDrill);
    window.addEventListener('drill-gamepad-update', updateFromGamepad);
    window.dispatchEvent(new CustomEvent('drill-subscribed'));
    return () => {
      window.removeEventListener('drill-subscribed', updateDrill);
      window.removeEventListener('drill-gamepad-update', updateFromGamepad);
    };
  }, []);

  useEffect(() => {
    setTelemetry(null);
    setWeight(null);
    setLastTelemetryAt(null);
    setLastWeightAt(null);
    ensureDrillTopics();

    const telemetryCb = (msg: DrillTelemetry) => {
      setTelemetry(msg);
      setLastTelemetryAt(Date.now());
    };
    const weightCb = (msg: { data: number }) => {
      setWeight(msg.data);
      setLastWeightAt(Date.now());
    };

    drillTelemetryTopic?.subscribe(telemetryCb);
    drillWeightTopic?.subscribe(weightCb);
    return () => {
      drillTelemetryTopic?.unsubscribe(telemetryCb);
      drillWeightTopic?.unsubscribe(weightCb);
    };
  }, [rerenderCount]);

  useEffect(() => {
    const interval = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(interval);
  }, []);

  const publishRack = (value: number) => {
    ensureDrillTopics();
    rackTopic?.publish({ data: normalizeInteger(value, RACK_MIN, RACK_MAX, 0) });
  };

  const publishDrill = (value: number) => {
    ensureDrillTopics();
    drillTopic?.publish({ data: normalizeInteger(value, DRILL_MIN, DRILL_MAX, 0) });
  };

  const commitRack = (rawValue?: unknown, skipBlur = false) => {
    const value = normalizeInteger(rawValue ?? rackInputRef.current?.getValue(), RACK_MIN, RACK_MAX, rackValue);
    rackSkipBlurRef.current = skipBlur;
    setRackValue(value);
    rackInputRef.current?.setValue(value);
    publishRack(value);
  };

  const commitDrill = (rawValue?: unknown, skipBlur = false) => {
    const value = normalizeInteger(rawValue ?? drillInputRef.current?.getValue(), DRILL_MIN, DRILL_MAX, drillValue);
    drillSkipBlurRef.current = skipBlur;
    setDrillValue(value);
    drillInputRef.current?.setValue(value);
    publishDrill(value);
  };

  const setServoInput = (channel: ServoChannel, rawValue: unknown) => {
    const angle = normalizeInteger(rawValue, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, servoAngles[channel]);
    setServoAngles((angles) => ({ ...angles, [channel]: angle }));
    servoInputRefs.current[channel]?.setValue(angle);
  };

  const commitServo = (channel: ServoChannel, rawValue?: unknown, skipBlur = false) => {
    const angle = normalizeInteger(
      rawValue ?? servoInputRefs.current[channel]?.getValue(),
      SERVO_MIN_ANGLE,
      SERVO_MAX_ANGLE,
      servoAngles[channel]
    );
    servoSkipBlurRefs.current[channel] = skipBlur;
    setServoInput(channel, angle);
    ensureDrillTopics();
    servoTopic?.publish({ data: [channel, angle] });
  };

  const stopRack = () => {
    setRackValue(0);
    rackInputRef.current?.setValue(0);
    publishRack(0);
  };

  const stopDrill = () => {
    setDrillValue(0);
    drillInputRef.current?.setValue(0);
    publishDrill(0);
  };

  const zeroServo = (channel: ServoChannel) => {
    setServoInput(channel, 0);
    ensureDrillTopics();
    servoTopic?.publish({ data: [channel, 0] });
  };

  const publishWeightCommand = (command: 0 | 1) => {
    ensureDrillTopics();
    weightCmdTopic?.publish({ data: command });
    setActiveWeightCommand(command);
    window.setTimeout(() => setActiveWeightCommand((active) => (active === command ? null : active)), 200);
  };

  const publishAutonomy = (state: AutonomyState) => {
    ensureDrillTopics();
    autonomyTopic?.publish({ data: state });
    setSelectedAutonomy(state);
  };

  const setRackDirection = (positive: boolean) => {
    const magnitude = Math.abs(normalizeInteger(rackInputRef.current?.getValue(), RACK_MIN, RACK_MAX, rackValue));
    commitRack(positive ? magnitude : -magnitude);
  };

  const setDrillDirection = (positive: boolean) => {
    const magnitude = Math.abs(normalizeInteger(drillInputRef.current?.getValue(), DRILL_MIN, DRILL_MAX, drillValue));
    commitDrill(positive ? magnitude : -magnitude);
  };

  const formatAge = (timestamp: number | null) => {
    if (timestamp === null) return 'Received ---';
    return `Received ${Math.max(0, Math.floor((now - timestamp) / 1000))} s ago`;
  };

  const autonomyStates = [
    { label: 'Stop', value: AutonomyState.Stop, icon: faStop, tooltip: 'Send stop command' },
    { label: 'Start', value: AutonomyState.Start, icon: faPlay, tooltip: 'Start autonomy' },
    {
      label: 'Autonomy Stop',
      value: AutonomyState.AutonomyStop,
      icon: faStop,
      tooltip: 'Stop autonomy (type 0x02)'
    },
    { label: 'Home', value: AutonomyState.Home, icon: faHouse, tooltip: 'Start homing' }
  ] as const;

  return (
    <div className={styles['drill-panel']}>
      <div className={styles['drill-controls']}>
        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Rack</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={rackValue > 0}
              tooltip='Move rack up'
              onClick={() => setRackDirection(true)}
            >
              <FontAwesomeIcon icon={faArrowUp} />
              &nbsp;&nbsp;Up
            </Button>
            <Button
              className={styles['large-button']}
              active={rackValue < 0}
              tooltip='Move rack down'
              onClick={() => setRackDirection(false)}
            >
              <FontAwesomeIcon icon={faArrowDown} />
              &nbsp;&nbsp;Down
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button
              className={styles['step-button']}
              tooltip='Decrease rack speed by 1 mm/sec'
              onClick={() => commitRack(rackValue - 1)}
            >
              <FontAwesomeIcon icon={faMinus} />
            </Button>
            <Input
              ref={rackInputRef}
              type='float'
              className={styles['speed-input']}
              defaultValue='0'
              onChange={(text) => setRackValue(normalizeInteger(text, RACK_MIN, RACK_MAX, rackValue))}
              onSubmit={(text) => commitRack(text, true)}
              onBlur={() => {
                if (rackSkipBlurRef.current) {
                  rackSkipBlurRef.current = false;
                  return;
                }
                commitRack();
              }}
            />
            <Button
              className={styles['step-button']}
              tooltip='Increase rack speed by 1 mm/sec'
              onClick={() => commitRack(rackValue + 1)}
            >
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send rack command' onClick={() => commitRack()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={`${styles['large-button']} ${styles['danger-button']}`}
              tooltip='Stop rack'
              onClick={stopRack}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Drill</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={drillValue < 0}
              tooltip='Rotate drill left'
              onClick={() => setDrillDirection(false)}
            >
              <FontAwesomeIcon icon={faArrowLeft} />
              &nbsp;&nbsp;Left
            </Button>
            <Button
              className={styles['large-button']}
              active={drillValue > 0}
              tooltip='Rotate drill right'
              onClick={() => setDrillDirection(true)}
            >
              <FontAwesomeIcon icon={faArrowRight} />
              &nbsp;&nbsp;Right
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button
              className={styles['step-button']}
              tooltip='Decrease drill duty by 1'
              onClick={() => commitDrill(drillValue - 1)}
            >
              <FontAwesomeIcon icon={faMinus} />
            </Button>
            <Input
              ref={drillInputRef}
              type='float'
              className={styles['speed-input']}
              defaultValue='0'
              onChange={(text) => setDrillValue(normalizeInteger(text, DRILL_MIN, DRILL_MAX, drillValue))}
              onSubmit={(text) => commitDrill(text, true)}
              onBlur={() => {
                if (drillSkipBlurRef.current) {
                  drillSkipBlurRef.current = false;
                  return;
                }
                commitDrill();
              }}
            />
            <Button
              className={styles['step-button']}
              tooltip='Increase drill duty by 1'
              onClick={() => commitDrill(drillValue + 1)}
            >
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send drill command' onClick={() => commitDrill()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={`${styles['large-button']} ${styles['danger-button']}`}
              tooltip='Stop drill'
              onClick={stopDrill}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Servo Set</div>
          {SERVO_CHANNELS.map((channel) => (
            <div className={styles['servo-row']} key={channel}>
              <Label color='var(--cyan-background)' className={styles['servo-label']}>
                {channel}
              </Label>
              <Input
                ref={(input) => {
                  servoInputRefs.current[channel] = input;
                }}
                type='float'
                className={styles['speed-input']}
                defaultValue='0'
                onChange={(text) => {
                  const angle = normalizeInteger(text, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, servoAngles[channel]);
                  setServoAngles((angles) => ({ ...angles, [channel]: angle }));
                }}
                onSubmit={(text) => commitServo(channel, text, true)}
                onBlur={() => {
                  if (servoSkipBlurRefs.current[channel]) {
                    servoSkipBlurRefs.current[channel] = false;
                    return;
                  }
                  setServoInput(channel, servoInputRefs.current[channel]?.getValue());
                }}
              />
              <Button
                className={styles['step-button']}
                tooltip={`Decrease servo ${channel} angle by 1°`}
                onClick={() => setServoInput(channel, servoAngles[channel] - 1)}
              >
                <FontAwesomeIcon icon={faMinus} />
              </Button>
              <Button
                className={styles['step-button']}
                tooltip={`Increase servo ${channel} angle by 1°`}
                onClick={() => setServoInput(channel, servoAngles[channel] + 1)}
              >
                <FontAwesomeIcon icon={faPlus} />
              </Button>
              <Button
                className={styles['icon-button']}
                tooltip={`Send servo ${channel} angle`}
                onClick={() => commitServo(channel)}
              >
                <FontAwesomeIcon icon={faPaperPlane} />
              </Button>
              <Button
                className={`${styles['icon-button']} ${styles['danger-button']}`}
                tooltip={`Set servo ${channel} to 0°`}
                onClick={() => zeroServo(channel)}
              >
                <FontAwesomeIcon icon={faStop} />
              </Button>
            </div>
          ))}
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Weight</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={activeWeightCommand === 0}
              tooltip='Tare the scale'
              onClick={() => publishWeightCommand(0)}
            >
              <FontAwesomeIcon icon={faWeightHanging} />
              &nbsp;&nbsp;Tare
            </Button>
            <Button
              className={styles['large-button']}
              active={activeWeightCommand === 1}
              tooltip='Request a weight measurement'
              onClick={() => publishWeightCommand(1)}
            >
              <FontAwesomeIcon icon={faArrowRotateRight} />
              &nbsp;&nbsp;Request
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Autonomy</div>
          <div className={styles['autonomy-grid']}>
            {autonomyStates.map((state) => (
              <Button
                key={state.value}
                className={
                  state.value === AutonomyState.AutonomyStop
                    ? `${styles['large-button']} ${styles['danger-button']}`
                    : styles['large-button']
                }
                active={selectedAutonomy === state.value}
                tooltip={state.tooltip}
                onClick={() => publishAutonomy(state.value)}
              >
                <FontAwesomeIcon icon={state.icon} />
                &nbsp;&nbsp;{state.label}
              </Button>
            ))}
          </div>
        </div>
      </div>

      <div className={styles['telemetry']}>
        <div className={styles['section-header']}>Telemetry</div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Depth
          </Label>
          <div className={`${styles['disabled-input']} ${styles['selectable']}`}>
            <input
              value={telemetry?.depth_mm !== undefined ? `${telemetry.depth_mm.toFixed(1)} mm` : '---'}
              disabled
              readOnly
            />
          </div>
        </div>
        <div className={styles['received-text']}>{formatAge(lastTelemetryAt)}</div>

        <div className={styles['section-header']}>Weight</div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Weight
          </Label>
          <div className={`${styles['disabled-input']} ${styles['selectable']}`}>
            <input value={weight !== null ? `${weight.toFixed(1)} g` : '---'} disabled readOnly />
          </div>
        </div>
        <div className={styles['received-text']}>{formatAge(lastWeightAt)}</div>
      </div>
    </div>
  );
}
