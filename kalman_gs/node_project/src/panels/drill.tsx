import styles from './drill.module.css';

import { ros } from '../common/ros';
import { DrillTelemetry } from '../common/ros-interfaces';
import {
  faArrowDown,
  faArrowLeft,
  faArrowRight,
  faArrowUp,
  faBoxOpen,
  faBroom,
  faDoorClosed,
  faDoorOpen,
  faHouse,
  faMinus,
  faPaperPlane,
  faPlus,
  faScrewdriverWrench,
  faStop,
  faTriangleExclamation
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

let drillBTopic: Topic<{ data: number }> | undefined;
let drillCTopic: Topic<{ data: number }> | undefined;
let drillStateTopic: Topic<{ data: number[] }> | undefined;
let drillShifterTopic: Topic<{ data: number[] }> | undefined;
let drillTelemetryTopic: Topic<DrillTelemetry> | undefined;

enum DrillState {
  STOP = 0,
  DRILLING_SITE_1 = 1,
  DRILLING_SITE_2 = 2,
  HOME = 3,
  CLOSING_TUBES_SITE_1 = 4,
  CLOSING_TUBES_SITE_2 = 5,
  CLEANING_DRILL = 6,
  OPENING_TUBES_SITE_1 = 7,
  OPENING_TUBES_SITE_2 = 8,
  OPENING_TUBES_BOTH_SITES = 9,
  RETRACT = 10
}

type DrillGamepadUpdate = {
  bridgeB?: number;
  bridgeC?: number;
  state?: DrillState;
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
  drillStateTopic ??= new Topic({
    ros,
    name: '/science/drill/state',
    messageType: 'std_msgs/UInt8MultiArray'
  });
  drillShifterTopic ??= new Topic({
    ros,
    name: '/science/drill/shifter',
    messageType: 'std_msgs/UInt8MultiArray'
  });
  drillTelemetryTopic ??= new Topic({
    ros,
    name: '/science/drill/telemetry',
    messageType: 'kalman_interfaces/DrillTelemetry'
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
  const [selectedState, setSelectedState] = useState<DrillState | null>(null);
  const [shifterStates, setShifterStates] = useState<[0 | 1 | null, 0 | 1 | null]>([null, null]);
  const [telemetry, setTelemetry] = useState<DrillTelemetry | null>(null);
  const [lastTelemetryAt, setLastTelemetryAt] = useState<number | null>(null);
  const [secondsSinceTelemetry, setSecondsSinceTelemetry] = useState<number | null>(null);
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
      if (detail.state !== undefined) {
        setSelectedState(detail.state);
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

    ensureDrillTopics();

    const telemetryTopic = drillTelemetryTopic;

    const telemetryCb = (msg: DrillTelemetry) => {
      setTelemetry(msg);
      setLastTelemetryAt(Date.now());
      setSecondsSinceTelemetry(0);
    };

    telemetryTopic?.subscribe(telemetryCb);
    return () => {
      telemetryTopic?.unsubscribe(telemetryCb);
    };
  }, [rerenderCount]);

  useEffect(() => {
    const interval = setInterval(() => {
      setSecondsSinceTelemetry(lastTelemetryAt === null ? null : Math.floor((Date.now() - lastTelemetryAt) / 1000));
    }, 1000);

    return () => clearInterval(interval);
  }, [lastTelemetryAt]);

  const site1States = [
    {
      label: 'Drill',
      value: DrillState.DRILLING_SITE_1,
      icon: faScrewdriverWrench,
      tooltip: 'Start drilling at site 1'
    },
    {
      label: 'Close',
      value: DrillState.CLOSING_TUBES_SITE_1,
      icon: faDoorClosed,
      tooltip: 'Close tubes at site 1'
    },
    {
      label: 'Open',
      value: DrillState.OPENING_TUBES_SITE_1,
      icon: faDoorOpen,
      tooltip: 'Open tubes at site 1'
    }
  ] as const;

  const site2States = [
    {
      label: 'Drill',
      value: DrillState.DRILLING_SITE_2,
      icon: faScrewdriverWrench,
      tooltip: 'Start drilling at site 2'
    },
    {
      label: 'Close',
      value: DrillState.CLOSING_TUBES_SITE_2,
      icon: faDoorClosed,
      tooltip: 'Close tubes at site 2'
    },
    {
      label: 'Open',
      value: DrillState.OPENING_TUBES_SITE_2,
      icon: faDoorOpen,
      tooltip: 'Open tubes at site 2'
    }
  ] as const;

  const generalStates = [
    { label: 'Stop', value: DrillState.STOP, icon: faStop, tooltip: 'Stop the drill' },
    { label: 'Home', value: DrillState.HOME, icon: faHouse, tooltip: 'Home the drill' },
    { label: 'Clean', value: DrillState.CLEANING_DRILL, icon: faBroom, tooltip: 'Clean the drill' },
    {
      label: 'Open Both',
      value: DrillState.OPENING_TUBES_BOTH_SITES,
      icon: faBoxOpen,
      tooltip: 'Open tubes at both sites'
    },
    {
      label: 'Retract',
      value: DrillState.RETRACT,
      icon: faTriangleExclamation,
      tooltip: 'Retract the drill'
    }
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

  const publishState = (rawState: number) => {
    const state = normalizeInteger(rawState, 0, 10, 0) as DrillState;

    setSelectedState(state);
    ensureDrillTopics();
    drillStateTopic?.publish({ data: [state] });
  };

  const publishShifter = (site: 0 | 1, state: 0 | 1) => {
    setShifterStates((current) => {
      const next = [...current] as [0 | 1 | null, 0 | 1 | null];
      next[site] = state;
      return next;
    });
    ensureDrillTopics();
    drillShifterTopic?.publish({ data: [site, state] });
  };

  const formatNumber = (value: number | undefined, digits = 1) => {
    return value !== undefined ? value.toFixed(digits) : '---';
  };

  const getFlag = (bit: number) => {
    if (telemetry?.flags === undefined) return undefined;
    return Boolean(telemetry.flags & (1 << bit));
  };

  const getBooleanColor = (value: boolean | undefined) => {
    if (value === undefined) return 'var(--dark-active)';
    return value ? 'var(--green-background)' : 'var(--red-background)';
  };

  const formatLimit = (value: boolean | undefined) => {
    if (value === undefined) return '---';
    return value ? 'Pressed' : 'Free';
  };

  const formatDrillState = (value: number | undefined) => {
    if (value === undefined) return '---';
    if (value === DrillState.STOP) return 'Stop';
    if (value === DrillState.DRILLING_SITE_1) return 'Drilling — Site 1';
    if (value === DrillState.DRILLING_SITE_2) return 'Drilling — Site 2';
    if (value === DrillState.HOME) return 'Home';
    if (value === DrillState.CLOSING_TUBES_SITE_1) return 'Closing tubes — Site 1';
    if (value === DrillState.CLOSING_TUBES_SITE_2) return 'Closing tubes — Site 2';
    if (value === DrillState.CLEANING_DRILL) return 'Cleaning drill';
    if (value === DrillState.OPENING_TUBES_SITE_1) return 'Opening tubes — Site 1';
    if (value === DrillState.OPENING_TUBES_SITE_2) return 'Opening tubes — Site 2';
    if (value === DrillState.OPENING_TUBES_BOTH_SITES) return 'Opening tubes — Both sites';
    if (value === DrillState.RETRACT) return 'Retract';
    if (value === 15) return 'Autonomy Disabled';
    return `Unknown ${value}`;
  };

  const renderStateButton = (state: (typeof site1States | typeof site2States | typeof generalStates)[number]) => (
    <Button
      key={state.value}
      className={styles['large-button']}
      active={selectedState === state.value}
      tooltip={state.tooltip}
      onClick={() => publishState(state.value)}
    >
      <FontAwesomeIcon icon={state.icon} />
      &nbsp;&nbsp;{state.label}
    </Button>
  );

  const renderBooleanStatus = (label: string, value: boolean | undefined) => (
    <Label color={getBooleanColor(value)} style={{ flex: 1 }}>
      {label}
    </Label>
  );

  return (
    <div className={styles['drill-panel']}>
      <div className={styles['drill-controls']}>
        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Rack</div>
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
              placeholder='Rack speed'
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
            <Button className={styles['large-button']} tooltip='Send rack command' onClick={() => commitBridgeB()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={styles['large-button'] + ' ' + styles['danger-button']}
              tooltip='Stop rack'
              onClick={stopBridgeB}
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
              active={cValue < 0}
              tooltip='Rotate drill left'
              onClick={() => setBridgeCDirection(false)}
            >
              <FontAwesomeIcon icon={faArrowLeft} />
              &nbsp;&nbsp;Left
            </Button>
            <Button
              className={styles['large-button']}
              active={cValue > 0}
              tooltip='Rotate drill right'
              onClick={() => setBridgeCDirection(true)}
            >
              <FontAwesomeIcon icon={faArrowRight} />
              &nbsp;&nbsp;Right
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
              placeholder='Drill speed'
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
            <Button className={styles['large-button']} tooltip='Send drill command' onClick={() => commitBridgeC()}>
              <FontAwesomeIcon icon={faPaperPlane} />
              &nbsp;&nbsp;Send
            </Button>
            <Button
              className={styles['large-button'] + ' ' + styles['danger-button']}
              tooltip='Stop drill'
              onClick={stopBridgeC}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;Stop
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Site 1</div>
          <div className={styles['button-row']}>{site1States.map(renderStateButton)}</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={shifterStates[0] === 0}
              tooltip='Open shifter at site 1'
              onClick={() => publishShifter(0, 0)}
            >
              <FontAwesomeIcon icon={faDoorOpen} />
              &nbsp;&nbsp;Open shifter
            </Button>
            <Button
              className={styles['large-button']}
              active={shifterStates[0] === 1}
              tooltip='Close shifter at site 1'
              onClick={() => publishShifter(0, 1)}
            >
              <FontAwesomeIcon icon={faDoorClosed} />
              &nbsp;&nbsp;Close shifter
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>Site 2</div>
          <div className={styles['button-row']}>{site2States.map(renderStateButton)}</div>
          <div className={styles['button-row']}>
            <Button
              className={styles['large-button']}
              active={shifterStates[1] === 0}
              tooltip='Open shifter at site 2'
              onClick={() => publishShifter(1, 0)}
            >
              <FontAwesomeIcon icon={faDoorOpen} />
              &nbsp;&nbsp;Open shifter
            </Button>
            <Button
              className={styles['large-button']}
              active={shifterStates[1] === 1}
              tooltip='Close shifter at site 2'
              onClick={() => publishShifter(1, 1)}
            >
              <FontAwesomeIcon icon={faDoorClosed} />
              &nbsp;&nbsp;Close shifter
            </Button>
          </div>
        </div>

        <div className={styles['bridge-section']}>
          <div className={styles['section-header']}>General</div>
          <div className={styles['state-grid']}>{generalStates.map(renderStateButton)}</div>
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
            Rack I
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={`${formatNumber(telemetry?.rack_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            Drill I
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
        <div className={styles['button-row']}>
          {renderBooleanStatus('Autonomy', telemetry?.autonomy_active)}
          {renderBooleanStatus('Based', telemetry?.based)}
        </div>
        <div className={styles['button-row']}>
          {renderBooleanStatus('1 Site I', getFlag(4))}
          {renderBooleanStatus('1 Site II', getFlag(5))}
        </div>
        <div className={styles['button-row']}>
          {renderBooleanStatus('2 Site I', getFlag(6))}
          {renderBooleanStatus('2 Site II', getFlag(7))}
        </div>
        <div className={styles['drill-row']}>
          <Label color='var(--dark-active)' className={styles['telemetry-label']}>
            State
          </Label>
          <div className={styles['disabled-input'] + ' ' + styles['selectable']}>
            <input value={formatDrillState(telemetry?.autonomy_state)} disabled readOnly />
          </div>
        </div>
        <div className={styles['received-text']}>
          {secondsSinceTelemetry === null ? 'Received ---' : `Received ${secondsSinceTelemetry} s ago`}
        </div>
      </div>
    </div>
  );
}
