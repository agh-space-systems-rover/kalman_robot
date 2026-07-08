import styles from './drill.module.css';

import { ros } from '../common/ros';
import { DrillTelemetry } from '../common/ros-interfaces';
import {
  faArrowDown,
  faArrowLeft,
  faArrowRight,
  faArrowRotateRight,
  faArrowUp,
  faBroom,
  faDoorClosed,
  faDoorOpen,
  faGear,
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
let drillGearTopic: Topic<{ data: number }> | undefined;
let drillTelemetryTopic: Topic<DrillTelemetry> | undefined;
let drillWeightReqTopic: Topic<{ data: number }> | undefined;
let drillWeightTopic: Topic<{ data: number }> | undefined;
let drillUniversalTopic: Topic<{ data: number }> | undefined;
let drillChannel1Topic: Topic<{ data: number }> | undefined;
let drillChannel2Topic: Topic<{ data: number }> | undefined;
let drillChannel3Topic: Topic<{ data: number }> | undefined;
let drillChannel4Topic: Topic<{ data: number }> | undefined;

type AutonomyState = 0 | 1 | 2 | 3 | 4 | 5;
type DrillGear = 1 | 2;
type UniversalChannel = 0 | 1 | 2 | 3 | 4;

type UniversalChannelConfig = {
  channel: UniversalChannel;
  label: string;
  max: number;
  placeholder: string;
};

const universalChannels: UniversalChannelConfig[] = [
  { channel: 0, label: 'Carousel', max: 100, placeholder: '0-100' },
  { channel: 1, label: 'Probe 1', max: 180, placeholder: '0-180' },
  { channel: 2, label: 'Probe 2', max: 180, placeholder: '0-180' },
  { channel: 3, label: 'Probe 3', max: 180, placeholder: '0-180' },
  { channel: 4, label: 'Probe 4', max: 180, placeholder: '0-180' }
];

const initialUniversalValues = universalChannels.reduce(
  (values, config) => ({
    ...values,
    [config.channel]: 0
  }),
  {} as Record<UniversalChannel, number>
);

type DrillGamepadUpdate = {
  bridgeB?: number;
  bridgeC?: number;
  autonomy?: AutonomyState;
  gear?: DrillGear;
  weightRequest?: 0 | 1;
};

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
  drillGearTopic ??= new Topic({
    ros,
    name: '/science/drill/gear',
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
  drillUniversalTopic ??= new Topic({
    ros,
    name: '/science/drill/universal',
    messageType: 'std_msgs/Float32'
  });
  drillChannel1Topic ??= new Topic({
    ros,
    name: '/science/drill/channel1',
    messageType: 'std_msgs/Float32'
  });
  drillChannel2Topic ??= new Topic({
    ros,
    name: '/science/drill/channel2',
    messageType: 'std_msgs/Float32'
  });
  drillChannel3Topic ??= new Topic({
    ros,
    name: '/science/drill/channel3',
    messageType: 'std_msgs/Float32'
  });
  drillChannel4Topic ??= new Topic({
    ros,
    name: '/science/drill/channel4',
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
  const universalInputRefs = useRef<Partial<Record<UniversalChannel, Input | null>>>({});
  const bSkipBlurRef = useRef(false);
  const cSkipBlurRef = useRef(false);
  const universalSkipBlurRefs = useRef<Partial<Record<UniversalChannel, boolean>>>({});
  const [bValue, setBValue] = useState(0);
  const [cValue, setCValue] = useState(0);
  const [universalValues, setUniversalValues] = useState<Record<UniversalChannel, number>>(initialUniversalValues);
  const [autonomyState, setAutonomyState] = useState<AutonomyState>(0);
  const [gear, setGear] = useState<DrillGear>(1);
  const [telemetry, setTelemetry] = useState<DrillTelemetry | null>(null);
  const [weight, setWeight] = useState<number | null>(null);
  const [lastTelemetryAt, setLastTelemetryAt] = useState<number | null>(null);
  const [secondsSinceTelemetry, setSecondsSinceTelemetry] = useState<number | null>(null);
  const [activeScaleAction, setActiveScaleAction] = useState<0 | 1 | null>(null);
  const [rerenderCount, setRerenderCount] = useState(0);

  useEffect(() => {
    const updateDrill = () => {
      setRerenderCount((count) => count + 1);
    };
    const updateFromGamepad = (event: Event) => {
      const detail = (event as CustomEvent<DrillGamepadUpdate>).detail;
      if (detail.bridgeB !== undefined) {
        setBValue(detail.bridgeB);
        bInputRef.current?.setValue(detail.bridgeB);
      }
      if (detail.bridgeC !== undefined) {
        setCValue(detail.bridgeC);
        cInputRef.current?.setValue(detail.bridgeC);
      }
      if (detail.autonomy !== undefined) {
        setAutonomyState(detail.autonomy);
      }
      if (detail.gear !== undefined) {
        setGear(detail.gear);
      }
      if (detail.weightRequest !== undefined) {
        setActiveScaleAction(detail.weightRequest);
        window.setTimeout(() => {
          setActiveScaleAction((current) => (current === detail.weightRequest ? null : current));
        }, 200);
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
    { label: 'Home', value: 2, icon: faArrowRotateRight, tooltip: 'Set drill autonomy to homing' },
    { label: 'Open Sarko', value: 3, icon: faDoorOpen, tooltip: 'Set drill autonomy to open sarko' },
    { label: 'Close Sarko', value: 4, icon: faDoorClosed, tooltip: 'Set drill autonomy to close sarko' },
    { label: 'Clean', value: 5, icon: faBroom, tooltip: 'Set drill autonomy to clean drill' }
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

  const getUniversalConfig = (channel: UniversalChannel) => {
    return universalChannels.find((config) => config.channel === channel)!;
  };

  const publishUniversalChannel = (channel: UniversalChannel, value: number) => {
    ensureDrillTopics();
    const topic =
      channel === 0
        ? drillUniversalTopic
        : channel === 1
          ? drillChannel1Topic
          : channel === 2
            ? drillChannel2Topic
            : channel === 3
              ? drillChannel3Topic
              : drillChannel4Topic;

    topic?.publish({ data: value });
  };

  const commitUniversalChannel = (channel: UniversalChannel, rawValue?: unknown, skipBlur = false) => {
    const config = getUniversalConfig(channel);
    const fallback = universalValues[channel];
    const nextValue = normalizeInteger(
      rawValue ?? universalInputRefs.current[channel]?.getValue(),
      0,
      config.max,
      fallback
    );

    universalSkipBlurRefs.current[channel] = skipBlur;
    setUniversalValues((values) => ({
      ...values,
      [channel]: nextValue
    }));
    universalInputRefs.current[channel]?.setValue(nextValue);
    publishUniversalChannel(channel, nextValue);
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
    const value = normalizeInteger(rawValue, 0, 5, 0) as AutonomyState;
    setAutonomyState(value);
    ensureDrillTopics();
    drillAutonomyTopic?.publish({ data: value });
  };

  const publishGear = (rawValue: number) => {
    const value = normalizeInteger(rawValue, 1, 2, 1) as DrillGear;
    setGear(value);
    ensureDrillTopics();
    drillGearTopic?.publish({ data: value });
  };

  const flashScaleAction = (value: 0 | 1) => {
    setActiveScaleAction(value);
    window.setTimeout(() => {
      setActiveScaleAction((current) => (current === value ? null : current));
    }, 200);
  };

  const requestWeight = () => {
    ensureDrillTopics();
    drillWeightReqTopic?.publish({ data: 1 });
    flashScaleAction(1);
  };

  const tareWeight = () => {
    ensureDrillTopics();
    drillWeightReqTopic?.publish({ data: 0 });
    flashScaleAction(0);
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
    if (value === 3) return 'Open Sarko';
    if (value === 4) return 'Close Sarko';
    if (value === 5) return 'Clean';
    return `Unknown ${value}`;
  };

  return (
    <div className={styles['drill-panel']}>
      <div className={styles['drill-controls']}>
        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Bridge B</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={bValue > 0}
              tooltip='Move rack up'
              onClick={() => setBridgeBDirection(true)}
            >
              <FontAwesomeIcon icon={faArrowUp} />
              &nbsp;&nbsp;Up
            </Button>
            <Button
              className={styles['large-button']}
              active={bValue < 0}
              tooltip='Move rack down'
              onClick={() => setBridgeBDirection(false)}
            >
              <FontAwesomeIcon icon={faArrowDown} />
              &nbsp;&nbsp;Down
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button
              className={styles['step-button']}
              tooltip='Decrease value by 1'
              onClick={() => commitBridgeB(bValue - 1)}
            >
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
            <Button
              className={styles['step-button']}
              tooltip='Increase value by 1'
              onClick={() => commitBridgeB(bValue + 1)}
            >
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send bridge B command' onClick={() => commitBridgeB()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={styles['large-button'] + ' ' + styles['danger-button']}
              tooltip='Stop bridge B'
              onClick={stopBridgeB}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Bridge C</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={cValue > 0}
              tooltip='Rotate drill right'
              onClick={() => setBridgeCDirection(true)}
            >
              <FontAwesomeIcon icon={faArrowRight} />
              &nbsp;&nbsp;Right
            </Button>
            <Button
              className={styles['large-button']}
              active={cValue < 0}
              tooltip='Rotate drill left'
              onClick={() => setBridgeCDirection(false)}
            >
              <FontAwesomeIcon icon={faArrowLeft} />
              &nbsp;&nbsp;Left
            </Button>
          </div>
          <div className={styles['input-row']}>
            <Button
              className={styles['step-button']}
              tooltip='Decrease value by 1'
              onClick={() => commitBridgeC(cValue - 1)}
            >
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
            <Button
              className={styles['step-button']}
              tooltip='Increase value by 1'
              onClick={() => commitBridgeC(cValue + 1)}
            >
              <FontAwesomeIcon icon={faPlus} />
            </Button>
          </div>
          <div className={styles['button-row']}>
            <Button className={styles['large-button']} tooltip='Send bridge C command' onClick={() => commitBridgeC()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={styles['large-button'] + ' ' + styles['danger-button']}
              tooltip='Stop bridge C'
              onClick={stopBridgeC}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Gear</div>
          <div className={styles['button-row']}>
            <Button
              className={`${styles['large-button']} ${
                gear === 1 ? styles['toggle-active'] : styles['toggle-inactive']
              }`}
              tooltip='Set drill gear 1'
              onClick={() => publishGear(1)}
            >
              <FontAwesomeIcon icon={faGear} />
              &nbsp;&nbsp;1
            </Button>
            <Button
              className={`${styles['large-button']} ${
                gear === 2 ? styles['toggle-active'] : styles['toggle-inactive']
              }`}
              tooltip='Set drill gear 2'
              onClick={() => publishGear(2)}
            >
              <FontAwesomeIcon icon={faGear} />
              &nbsp;&nbsp;2
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Universal</div>
          {universalChannels.map((config) => (
            <div className={styles['universal-row']} key={config.channel}>
              <Label color='var(--dark-active)' className={styles['universal-label']}>
                {config.label}
              </Label>
              <Input
                ref={(input) => {
                  universalInputRefs.current[config.channel] = input;
                }}
                type='float'
                className={styles['speed-input']}
                placeholder={config.placeholder}
                defaultValue={String(universalValues[config.channel])}
                onChange={(text) => {
                  const value = normalizeInteger(text, 0, config.max, universalValues[config.channel]);
                  setUniversalValues((values) => ({
                    ...values,
                    [config.channel]: value
                  }));
                }}
                onSubmit={(text) => commitUniversalChannel(config.channel, text, true)}
                onBlur={() => {
                  if (universalSkipBlurRefs.current[config.channel]) {
                    universalSkipBlurRefs.current[config.channel] = false;
                    return;
                  }
                  commitUniversalChannel(config.channel);
                }}
              />
              <Button
                className={styles['step-button']}
                tooltip={`Send ${config.label} command`}
                onClick={() => commitUniversalChannel(config.channel)}
              >
                <FontAwesomeIcon icon={faPaperPlane} />
              </Button>
            </div>
          ))}
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Scale</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={activeScaleAction === 1}
              tooltip='Request drill weight measurement'
              onClick={requestWeight}
            >
              <FontAwesomeIcon icon={faWeightHanging} />
              &nbsp;&nbsp;Weigh
            </Button>
            <Button
              className={styles['large-button']}
              active={activeScaleAction === 0}
              tooltip='Tare drill scale'
              onClick={tareWeight}
            >
              <FontAwesomeIcon icon={faWeightHanging} />
              &nbsp;&nbsp;Tare
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Autonomy</div>
          <div className={styles['autonomy-grid']}>
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
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Depth
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.depth_mm)} mm`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Rack Current I
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.rack_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Drill Current I
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.drill_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Upper Limit
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatLimit(telemetry?.upper_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Lower Limit
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatLimit(telemetry?.lower_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Autonomy
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatAutonomyActive(telemetry?.autonomy_active)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Based
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatBased(telemetry?.based)} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            State
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatAutonomyState(telemetry?.autonomy_state)} disabled readOnly />
          </div>
        </div>
        <div className={styles['received-text']}>
          {secondsSinceTelemetry === null ? 'Received ---' : `Received ${secondsSinceTelemetry} s ago`}
        </div>

        <div className={styles['section-header']}>Scale</div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Weight
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={weight !== null ? `${weight.toFixed(2)} g` : '---'} disabled readOnly />
          </div>
        </div>
      </div>
    </div>
  );
}
