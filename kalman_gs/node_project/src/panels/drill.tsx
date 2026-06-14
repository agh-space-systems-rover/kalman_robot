import styles from './drill.module.css';

import { ros } from '../common/ros';
import { DrillTelemetry } from '../common/ros-interfaces';
import {
  faArrowDown,
  faArrowLeft,
  faArrowRight,
  faArrowRotateRight,
  faArrowUp,
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

let drillBTopic: Topic<{ data: number }> | undefined;
let drillCTopic: Topic<{ data: number }> | undefined;
let drillAutonomyTopic: Topic<{ data: number }> | undefined;
let drillTelemetryTopic: Topic<DrillTelemetry> | undefined;
let drillWeightReqTopic: Topic<{ data: number }> | undefined;
let drillWeightTopic: Topic<{ data: number }> | undefined;

const ensureDrillTopics = () => {
  if (!ros.isConnected) return;

  drillBTopic ??= new Topic({
    ros,
    name: '/science/drill/b',
    messageType: 'std_msgs/Int8'
  });
  drillCTopic ??= new Topic({
    ros,
    name: '/science/drill/c',
    messageType: 'std_msgs/Int8'
  });
  drillAutonomyTopic ??= new Topic({
    ros,
    name: '/science/drill/autonomy',
    messageType: 'std_msgs/UInt8'
  });
  drillTelemetryTopic ??= new Topic({
    ros,
    name: '/science/drill/telemetry',
    messageType: 'kalman_interfaces/DrillTelemetry'
  });
  drillWeightReqTopic ??= new Topic({
    ros,
    name: '/science/drill/weight/request',
    messageType: 'std_msgs/UInt8'
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

export default function Drill() {
  const bInputRef = useRef<Input>(null);
  const cInputRef = useRef<Input>(null);
  const bSkipBlurRef = useRef(false);
  const cSkipBlurRef = useRef(false);
  const [bValue, setBValue] = useState(0);
  const [cValue, setCValue] = useState(0);
  const [autonomyState, setAutonomyState] = useState(0);
  const [telemetry, setTelemetry] = useState<DrillTelemetry | null>(null);
  const [weight, setWeight] = useState<number | null>(null);
  const [lastTelemetryAt, setLastTelemetryAt] = useState<number | null>(null);
  const [secondsSinceTelemetry, setSecondsSinceTelemetry] = useState<number | null>(null);
  const [rerenderCount, setRerenderCount] = useState(0);

  useEffect(() => {
    const updateDrill = () => {
      setRerenderCount((count) => count + 1);
    };
    ensureDrillTopics();
    window.addEventListener('drill-subscribed', updateDrill);
    window.dispatchEvent(new CustomEvent('drill-subscribed'));
    return () => {
      window.removeEventListener('drill-subscribed', updateDrill);
    };
  }, []);

  useEffect(() => {
    setTelemetry(null);
    setLastTelemetryAt(null);
    setSecondsSinceTelemetry(null);
    setWeight(null);

    ensureDrillTopics();

    const telemetryTopic = drillTelemetryTopic;
    const weightTopic = drillWeightTopic;

    const telemetryCb = (msg: DrillTelemetry) => {
      setTelemetry(msg);
      setLastTelemetryAt(Date.now());
      setSecondsSinceTelemetry(0);
    };
    const weightCb = (msg: { data: number }) => {
      setWeight(msg.data);
    };

    telemetryTopic?.subscribe(telemetryCb);
    weightTopic?.subscribe(weightCb);
    return () => {
      telemetryTopic?.unsubscribe(telemetryCb);
      weightTopic?.unsubscribe(weightCb);
    };
  }, [rerenderCount]);

  useEffect(() => {
    const interval = setInterval(() => {
      setSecondsSinceTelemetry(lastTelemetryAt === null ? null : Math.floor((Date.now() - lastTelemetryAt) / 1000));
    }, 1000);

    return () => clearInterval(interval);
  }, [lastTelemetryAt]);

  const autonomyStates = [
    { label: 'Stop', value: 0, icon: faStop, tooltip: 'Set drill autonomy to emergency stop' },
    { label: 'Drill', value: 1, icon: faPlay, tooltip: 'Set drill autonomy to drilling' },
    { label: 'Home', value: 2, icon: faArrowRotateRight, tooltip: 'Set drill autonomy to homing' }
  ] as const;

  const normalizeInteger = (value: unknown, min: number, max: number, fallback: number) => {
    const parsedValue = typeof value === 'number' ? value : parseFloat(String(value));
    if (!Number.isFinite(parsedValue)) {
      return fallback;
    }
    return Math.max(min, Math.min(Math.round(parsedValue), max));
  };

  const publishBridge = (getTopic: () => Topic<{ data: number }> | undefined, value: number) => {
    ensureDrillTopics();
    getTopic()?.publish({ data: normalizeInteger(value, -100, 100, 0) });
  };

  const commitBridgeB = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeInteger(rawValue ?? bInputRef.current?.getValue(), -100, 100, bValue);
    bSkipBlurRef.current = skipBlur;
    setBValue(nextValue);
    bInputRef.current?.setValue(nextValue);
    publishBridge(() => drillBTopic, nextValue);
  };

  const commitBridgeC = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeInteger(rawValue ?? cInputRef.current?.getValue(), -100, 100, cValue);
    cSkipBlurRef.current = skipBlur;
    setCValue(nextValue);
    cInputRef.current?.setValue(nextValue);
    publishBridge(() => drillCTopic, nextValue);
  };

  const setBridgeBDirection = (positive: boolean) => {
    const magnitude = Math.abs(normalizeInteger(bInputRef.current?.getValue(), -100, 100, bValue));
    commitBridgeB(positive ? magnitude : -magnitude);
  };

  const setBridgeCDirection = (positive: boolean) => {
    const magnitude = Math.abs(normalizeInteger(cInputRef.current?.getValue(), -100, 100, cValue));
    commitBridgeC(positive ? magnitude : -magnitude);
  };

  const stopBridgeB = () => {
    bSkipBlurRef.current = false;
    setBValue(0);
    bInputRef.current?.setValue(0);
    publishBridge(() => drillBTopic, 0);
  };

  const stopBridgeC = () => {
    cSkipBlurRef.current = false;
    setCValue(0);
    cInputRef.current?.setValue(0);
    publishBridge(() => drillCTopic, 0);
  };

  const publishAutonomy = (rawValue: number) => {
    const value = normalizeInteger(rawValue, 0, 2, 0);
    setAutonomyState(value);
    ensureDrillTopics();
    drillAutonomyTopic?.publish({ data: value });
  };

  const requestWeight = () => {
    ensureDrillTopics();
    drillWeightReqTopic?.publish({ data: 1 });
  };

  const tareWeight = () => {
    ensureDrillTopics();
    drillWeightReqTopic?.publish({ data: 0 });
  };

  const formatNumber = (value: number | undefined, digits = 1) => {
    return value !== undefined ? value.toFixed(digits) : '---';
  };

  const formatLimit = (value: boolean | undefined) => {
    if (value === undefined) return '---';
    return value ? 'Pressed' : 'Free';
  };

  const formatAutonomyActive = (value: boolean | undefined) => {
    if (value === undefined) return '---';
    return value ? 'Active' : 'Manual';
  };

  const formatBased = (value: boolean | undefined) => {
    if (value === undefined) return '---';
    return value ? 'Ready' : 'Required';
  };

  const formatAutonomyState = (value: number | undefined) => {
    if (value === undefined) return '---';
    if (value === 0) return 'Stop';
    if (value === 1) return 'Drilling';
    if (value === 2) return 'Homing';
    return `Unknown ${value}`;
  };

  return (
    <div className={styles['drill-panel']}>
      <div className={styles['drill-controls']}>
        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Bridge B</div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Move rack up' onClick={() => setBridgeBDirection(true)}>
              <FontAwesomeIcon icon={faArrowUp} />
              &nbsp;&nbsp;Up
            </Button>
            <Button className={styles['large-button']} tooltip='Move rack down' onClick={() => setBridgeBDirection(false)}>
              <FontAwesomeIcon icon={faArrowDown} />
              &nbsp;&nbsp;Down
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button className={styles['step-button']} tooltip='Decrease value by 1' onClick={() => commitBridgeB(bValue - 1)}>
              <FontAwesomeIcon icon={faMinus} />
            </Button>
            <Input
              ref={bInputRef}
              type='float'
              className={styles['speed-input']}
              placeholder='Bridge B speed'
              defaultValue={String(bValue)}
              onChange={(text) => setBValue(normalizeInteger(text, -100, 100, bValue))}
              onSubmit={(text) => commitBridgeB(text, true)}
              onBlur={() => {
                if (bSkipBlurRef.current) {
                  bSkipBlurRef.current = false;
                  return;
                }
                commitBridgeB();
              }}
            />
            <Button className={styles['step-button']} tooltip='Increase value by 1' onClick={() => commitBridgeB(bValue + 1)}>
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send bridge B command' onClick={() => commitBridgeB()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button className={styles['large-button'] + ' ' + styles['danger-button']} tooltip='Stop bridge B' onClick={stopBridgeB}>
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Bridge C</div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Rotate drill right' onClick={() => setBridgeCDirection(true)}>
              <FontAwesomeIcon icon={faArrowRight} />
              &nbsp;&nbsp;Right
            </Button>
            <Button className={styles['large-button']} tooltip='Rotate drill left' onClick={() => setBridgeCDirection(false)}>
              <FontAwesomeIcon icon={faArrowLeft} />
              &nbsp;&nbsp;Left
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button className={styles['step-button']} tooltip='Decrease value by 1' onClick={() => commitBridgeC(cValue - 1)}>
              <FontAwesomeIcon icon={faMinus} />
            </Button>
            <Input
              ref={cInputRef}
              type='float'
              className={styles['speed-input']}
              placeholder='Bridge C speed'
              defaultValue={String(cValue)}
              onChange={(text) => setCValue(normalizeInteger(text, -100, 100, cValue))}
              onSubmit={(text) => commitBridgeC(text, true)}
              onBlur={() => {
                if (cSkipBlurRef.current) {
                  cSkipBlurRef.current = false;
                  return;
                }
                commitBridgeC();
              }}
            />
            <Button className={styles['step-button']} tooltip='Increase value by 1' onClick={() => commitBridgeC(cValue + 1)}>
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send bridge C command' onClick={() => commitBridgeC()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button className={styles['large-button'] + ' ' + styles['danger-button']} tooltip='Stop bridge C' onClick={stopBridgeC}>
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Scale</div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Request drill weight measurement' onClick={requestWeight}>
              <FontAwesomeIcon icon={faWeightHanging} />
              &nbsp;&nbsp;Weigh
            </Button>
            <Button className={styles['large-button']} tooltip='Tare drill scale' onClick={tareWeight}>
              <FontAwesomeIcon icon={faWeightHanging} />
              &nbsp;&nbsp;Tare
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Autonomy</div>
          <div className={styles['button-row']}>
            {autonomyStates.map((state) => (
              <Button
                key={state.value}
                className={`${styles['large-button']} ${
                  autonomyState === state.value ? styles['toggle-active'] : styles['toggle-inactive']
                }`}
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
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Depth</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.depth_mm)} mm`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Rack Current I</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.rack_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Drill Current I</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.drill_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Upper Limit</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatLimit(telemetry?.upper_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Lower Limit</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatLimit(telemetry?.lower_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Autonomy</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatAutonomyActive(telemetry?.autonomy_active)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Based</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatBased(telemetry?.based)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>State</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatAutonomyState(telemetry?.autonomy_state)} disabled readOnly />
          </div>
        </div>
        <div className={styles['received-text']}>
          {secondsSinceTelemetry === null ? 'Received ---' : `Received ${secondsSinceTelemetry} s ago`}
        </div>

        <div className={styles['section-header']}>Scale</div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>Weight</Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={weight !== null ? `${weight.toFixed(2)} g` : '---'} disabled readOnly />
          </div>
        </div>
      </div>
    </div>
  );
}
