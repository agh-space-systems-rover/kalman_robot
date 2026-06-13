import styles from './science.module.css';



import { modalRef } from '../common/refs';
import { ros } from '../common/ros';
import { DrillTelemetry } from '../common/ros-interfaces';
import { faWeightHanging, faFlask, faBoxOpen, faBox, faArrowRotateRight, faDroplet, faTrash, faList, faBan, faMagnet, faArrowDown, faArrowUp, faArrowLeft, faArrowRight, faMinus, faPlus, faStop, faDiagramProject, faRuler, faOilWell, faPlay, faPaperPlane } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState, useRef } from 'react';
import { Topic, Service } from 'roslib';



import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';


const STORAGE_OPTIONS = [
  { name: 'Sand Storage', value: 'sand', icon: faFlask },
  { name: 'Rock Storage', value: 'rock', icon: faBox },
  { name: 'pH Probe', value: 'ph', icon: faDroplet },
  { name: 'Magnetometer', value: 'magneto', icon: faMagnet },
  { name: 'Drill', value: 'drill', icon: faOilWell }
];
// Drill ROS clients
let drillBTopic: any;
let drillCTopic: any;
let drillAutonomyTopic: any;
let drillTelemetryTopic: any;

// Global ROS clients
let sandWeightTopic: any;
let rockWeightTopic: any;
let phValueTopic: any;
let phRailTargetVelTopic: any;
let magnetometerTopic: any;
let sandWeightService: any;
let rockWeightService: any;
let sandOpenService: any;
let sandCloseService: any;
let rockOpenService: any;
let rockCloseService: any;
let phValueService: any;
let magnetometerResetService: any;

window.addEventListener('ros-connect', () => {
  // Weight topics
  sandWeightTopic = new Topic({
    ros,
    name: '/science/storage/sand/weight',
    messageType: 'std_msgs/Float32'
  });
  rockWeightTopic = new Topic({
    ros,
    name: '/science/storage/rock/weight',
    messageType: 'std_msgs/Float32'
  });
  phValueTopic = new Topic({
    ros,
    name: '/science/ph/value',
    messageType: 'std_msgs/Float32'
  });
  phRailTargetVelTopic = new Topic({
    ros,
    name: '/science/ph/rail/target_vel',
    messageType: 'std_msgs/Float32'
  });

  magnetometerTopic = new Topic({
    ros,
    name: '/science/magnetic_field/value',
    messageType: 'sensor_msgs/MagneticField'
  });

  // Weight request services
  sandWeightService = new Service({
    ros,
    name: '/science/storage/sand/weight/req',
    serviceType: 'std_srvs/Trigger'
  });
  rockWeightService = new Service({
    ros,
    name: '/science/storage/rock/weight/req',
    serviceType: 'std_srvs/Trigger'
  });

  // Storage container services
  sandOpenService = new Service({
    ros,
    name: '/science/storage/sand/open',
    serviceType: 'std_srvs/Trigger'
  });
  sandCloseService = new Service({
    ros,
    name: '/science/storage/sand/close',
    serviceType: 'std_srvs/Trigger'
  });
  rockOpenService = new Service({
    ros,
    name: '/science/storage/rock/open',
    serviceType: 'std_srvs/Trigger'
  });
  rockCloseService = new Service({
    ros,
    name: '/science/storage/rock/close',
    serviceType: 'std_srvs/Trigger'
  });

  // pH service
  phValueService = new Service({
    ros,
    name: '/science/ph/value/req',
    serviceType: 'std_srvs/Trigger'
  });

  magnetometerResetService = new Service({
    ros,
    name: '/science/magnetic_field/value/req',
    serviceType: 'std_srvs/Trigger'
  });

  // Drill topics
  drillBTopic = new Topic({
    ros,
    name: '/science/drill/b',
    messageType: 'std_msgs/Int8'
  });
  drillCTopic = new Topic({
    ros,
    name: '/science/drill/c',
    messageType: 'std_msgs/Int8'
  });
  drillAutonomyTopic = new Topic({
    ros,
    name: '/science/drill/autonomy',
    messageType: 'std_msgs/UInt8'
  });
  drillTelemetryTopic = new Topic({
    ros,
    name: '/science/drill/telemetry',
    messageType: 'kalman_interfaces/DrillTelemetry'
  });

  window.dispatchEvent(new CustomEvent('science-subscribed'));
});

type SciencePanelProps = {
  props: {
    selectedStorage: string;
    sandTareHistory?: number[];
    rockTareHistory?: number[];
    drillTareHistory?: number[];
  };
};

type StorageContainerProps = {
  selectedStorage: string;
  tareHistory: number[];
  onTareHistoryChange: (history: number[]) => void;
};

type PHProbeProps = {};

export default function Science({ props }: SciencePanelProps) {
  if (props.selectedStorage === undefined) {
    props.selectedStorage = 'sand';
  }
  if (props.sandTareHistory === undefined) {
    props.sandTareHistory = [];
  }
  if (props.rockTareHistory === undefined) {
    props.rockTareHistory = [];
  }
  if (props.drillTareHistory === undefined) {
    props.drillTareHistory = [];
  }

  const [selectedStorage, setSelectedStorage] = useState(props.selectedStorage);
  const [rerenderCount, setRerenderCount] = useState(0);

  const getTareHistory = () => {
    if (selectedStorage === 'sand') return props.sandTareHistory;
    if (selectedStorage === 'rock') return props.rockTareHistory;
    if (selectedStorage === 'drill') return props.drillTareHistory;
    return [];
  };

  const setTareHistory = (history: number[]) => {
    if (selectedStorage === 'sand') {
      props.sandTareHistory = history;
    } else if (selectedStorage === 'rock') {
      props.rockTareHistory = history;
    } else if (selectedStorage === 'drill') {
      props.drillTareHistory = history;
    }
    setRerenderCount(rerenderCount + 1);
  };

  return (
    <div className={`${styles['science']} ${selectedStorage === 'drill' ? styles['science-drill'] : ''}`}>
      <div className={`${styles['science-rows']} ${selectedStorage === 'drill' ? styles['science-drill-rows'] : ''}`}>
        <div className={styles['science-row']}>
          <Dropdown
            className={styles['science-row-item']}
            tooltip='Select science instrument type'
            items={STORAGE_OPTIONS.map((opt) => ({
              icon: opt.icon,
              text: opt.name
            }))}
            defaultItemIndex={STORAGE_OPTIONS.findIndex((opt) => opt.value === selectedStorage)}
            onChange={(i) => {
              const newStorage = STORAGE_OPTIONS[i].value;
              setSelectedStorage(newStorage);
              props.selectedStorage = newStorage;
            }}
          />
        </div>

        {(selectedStorage === 'sand' || selectedStorage === 'rock') && (
          <StorageContainer
            selectedStorage={selectedStorage}
            tareHistory={getTareHistory()}
            onTareHistoryChange={setTareHistory}
          />
        )}

        {selectedStorage === 'drill' && <DrillContainer />}

        {selectedStorage === 'ph' && <PHProbe />}
        {selectedStorage === 'magneto' && <Magnetometer />}
      </div>
    </div>
  );
}

function StorageContainer({ selectedStorage, tareHistory, onTareHistoryChange }: StorageContainerProps) {
  const [weight, setWeight] = useState<number | null>(null);
  const [rerenderCount, setRerenderCount] = useState(0);

  // Rerender on science-subscribed event
  useEffect(() => {
    const updateScience = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('science-subscribed', updateScience);
    return () => {
      window.removeEventListener('science-subscribed', updateScience);
    };
  }, []);

  // Subscribe to weight topic
  useEffect(() => {
    setWeight(null);

    const topic = selectedStorage === 'sand' ? sandWeightTopic : rockWeightTopic;
    if (!topic) return;

    const cb = (msg: { data: number }) => {
      setWeight(msg.data);
    };

    topic.subscribe(cb);
    return () => topic.unsubscribe(cb);
  }, [selectedStorage, rerenderCount]);

  // Service call helpers
  function callStorageService(action: string) {
    let service;
    if (selectedStorage === 'sand') {
      service = action === 'open' ? sandOpenService : action === 'close' ? sandCloseService : sandWeightService;
    } else {
      service = action === 'open' ? rockOpenService : action === 'close' ? rockCloseService : rockWeightService;
    }

    if (service) {
      service.callService({}, () => {});
    }
  }

  function handleTare() {
    if (weight !== null) {
      const currentTareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
      const tareValue = weight - currentTareOffset; // Current tared reading
      const newHistory = [...tareHistory, tareValue];
      onTareHistoryChange(newHistory);
    }
  }

  function clearTareHistory() {
    onTareHistoryChange([]);
  }

  const tareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
  const displayWeight = weight !== null ? weight - tareOffset : null;

  const style = getComputedStyle(document.body);
  const greenBg = style.getPropertyValue('--green-background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <>
      <div className={styles['science-row']}>
        <Button
          className={styles['science-row-item']}
          tooltip='Open storage container'
          onClick={() => callStorageService('open')}
        >
          <FontAwesomeIcon icon={faBoxOpen} />
          &nbsp;&nbsp;Open&nbsp;
        </Button>
        <Button
          className={styles['science-row-item']}
          tooltip='Close storage container'
          onClick={() => callStorageService('close')}
        >
          <FontAwesomeIcon icon={faBox} />
          &nbsp;&nbsp;Close&nbsp;
        </Button>
      </div>

      <div className={styles['science-row']}>
        <Label color={greenBg}>
          <FontAwesomeIcon icon={faWeightHanging} />
        </Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {displayWeight !== null ? `${displayWeight.toFixed(2)} g` : '---'}
        </Label>
        <Button tooltip='Refresh weight measurement' onClick={() => callStorageService('weight')}>
          <FontAwesomeIcon icon={faArrowRotateRight} />
        </Button>
      </div>

      <div className={styles['science-row']}>
        <Button
          className={styles['science-row-item']}
          tooltip='Set current weight as zero (tare)'
          onClick={handleTare}
          disabled={weight === null}
        >
          <FontAwesomeIcon icon={faWeightHanging} />
          &nbsp;&nbsp;Tare
        </Button>
      </div>

      <div className={styles['science-row']}>
        <div className={styles['tare-history']}>
          <div className={styles['science-row']}>
            <Label className={styles['tare-history-header']}>
              <FontAwesomeIcon icon={tareHistory.length > 0 ? faList : faBan} />
              &nbsp; {tareHistory.length > 0 ? '' : 'No'} Tare History
            </Label>
          </div>
          {tareHistory.map((tareValue, index) => (
            <div key={index} className={styles['science-row']}>
              <Label className={styles['tare-entry']}>
                {index + 1}. {tareValue.toFixed(2)} g
              </Label>
            </div>
          ))}
        </div>
      </div>

      {tareHistory.length > 0 && (
        <div className={styles['science-row']}>
          <Button
            className={styles['science-row-item']}
            tooltip='Clear all tare history'
            onClick={() => {
              modalRef.current?.showConfirm({
                title: 'Clear tare history',
                icon: faTrash,
                message: 'Are you sure you want to clear all tare history?',
                confirmText: 'Clear',
                cancelText: 'Cancel',
                onConfirm: clearTareHistory
              });
            }}
          >
            <FontAwesomeIcon icon={faTrash} />
            &nbsp;&nbsp;Clear History
          </Button>
        </div>
      )}
    </>
  );
}

function PHProbe({}: PHProbeProps) {
  const [phValue, setPhValue] = useState<number | null>(null);
  const [moveSpeed, setMoveSpeed] = useState(0);
  const [rerenderCount, setRerenderCount] = useState(0);
  const sentZerosRef = useRef(0);

  const MAX_MOVE_SPEED = 3;

  // Rerender on science-subscribed event
  useEffect(() => {
    const updateScience = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('science-subscribed', updateScience);
    return () => {
      window.removeEventListener('science-subscribed', updateScience);
    };
  }, []);

  // Subscribe to pH value topic
  useEffect(() => {
    setPhValue(null);

    if (!phValueTopic) return;

    const cb = (msg: { data: number }) => {
      setPhValue(msg.data);
    };

    phValueTopic.subscribe(cb);
    return () => phValueTopic.unsubscribe(cb);
  }, [rerenderCount]);

  // Send rail speed every 100ms
  useEffect(() => {
    sentZerosRef.current = 0;

    const interval = setInterval(() => {
      if (!phRailTargetVelTopic) return;
      const MOVE_SPEED_VALUES = [-1.0, -0.6, -0.2, 0.0, 0.2, 0.6, 1.0];
      const speed = MOVE_SPEED_VALUES[moveSpeed + MAX_MOVE_SPEED];
      if (Math.abs(speed) < 0.01) {
        if (sentZerosRef.current >= 5) return; // Don't spam zeros
        phRailTargetVelTopic.publish({ data: 0.0 });
        sentZerosRef.current++;
      } else {
        phRailTargetVelTopic.publish({ data: speed });
        sentZerosRef.current = 0;
      }
    }, 100);

    return () => clearInterval(interval);
  }, [moveSpeed]);

  // Service call helper
  function requestPhMeasurement() {
    if (phValueService) {
      phValueService.callService({}, () => {});
    }
  }

  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const yellowBg = style.getPropertyValue('--yellow-background');
  const greenBg = style.getPropertyValue('--green-background');
  const blueBg = style.getPropertyValue('--blue-background');
  const darkBg = style.getPropertyValue('--dark-background');
  const bgColor = style.getPropertyValue('--background');
  const activeColor = style.getPropertyValue('--active');

  return (
    <>
      <div className={styles['science-row']}>
        <Label color={blueBg}>
          <FontAwesomeIcon icon={faDroplet} />
        </Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {phValue !== null ? `${phValue.toFixed(2)} pH` : '---'}
        </Label>
      </div>
      <div className={styles['science-row']}>
        <Button
          className={styles['science-row-item']}
          tooltip='Request new pH measurement'
          onClick={requestPhMeasurement}
        >
          <FontAwesomeIcon icon={faArrowRotateRight} />
          &nbsp;&nbsp;
          <span style={{ marginTop: '2px' }}>Refresh</span>
        </Button>
      </div>
      <div className={styles['science-row']}>
        <div className={styles['tare-history']}>
          <div className={styles['science-row']}>
            <Label color={darkBg} className={styles['science-row-item']}>
              <FontAwesomeIcon icon={faDiagramProject} />
              &nbsp;&nbsp; Movement Speed
            </Label>
          </div>
          <div className={styles['science-row']}>
            {Array.from({ length: MAX_MOVE_SPEED * 2 + 1 }, (_, i) => {
              const speed = -MAX_MOVE_SPEED + i;
              const isActive = speed === moveSpeed;
              let color;
              if (isActive) {
                color = speed > 0 ? greenBg : speed < 0 ? redBg : activeColor;
              } else {
                color = bgColor;
              }
              // Saturation modifier: more extreme values get more saturated color
              const saturation = Math.abs(speed) / MAX_MOVE_SPEED;
              const style = isActive && speed !== 0 ? { filter: `saturate(${saturation})` } : {};

              return (
                <Label key={speed} color={color} className={styles['science-row-item']} style={style}>
                  {null}
                </Label>
              );
            })}
          </div>
          <div className={styles['science-row']}>
            <Button
              className={styles['science-row-item']}
              tooltip='Decrease movement speed'
              onClick={() => setMoveSpeed((speed) => Math.max(-MAX_MOVE_SPEED, speed - 1))}
            >
              <FontAwesomeIcon icon={faArrowDown} />
              &nbsp;&nbsp;
              <span style={{ marginTop: '2px' }}>Down</span>
            </Button>
            <Button
              className={styles['science-row-item']}
              tooltip='Stop movement'
              onClick={() => {
                setMoveSpeed(0);
                sentZerosRef.current = 0;
              }}
            >
              <FontAwesomeIcon icon={faStop} />
              &nbsp;&nbsp;
              <span style={{ marginTop: '2px' }}>Stop</span>
            </Button>
            <Button
              className={styles['science-row-item']}
              tooltip='Increase movement speed'
              onClick={() => setMoveSpeed((speed) => Math.min(MAX_MOVE_SPEED, speed + 1))}
            >
              <FontAwesomeIcon icon={faArrowUp} />
              &nbsp;&nbsp;
              <span style={{ marginTop: '2px' }}>Up</span>
            </Button>
          </div>
        </div>
      </div>
    </>
  );
}

function Magnetometer() {
  const [magField, setMagField] = useState(null);
  const [rerenderCount, setRerenderCount] = useState(0);

  useEffect(() => {
    const updateScience = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('science-subscribed', updateScience);
    return () => {
      window.removeEventListener('science-subscribed', updateScience);
    };
  }, []);

  useEffect(() => {
    setMagField(null);
    if (!magnetometerTopic) return;
    const cb = (msg) => {
      setMagField({
        x: msg.magnetic_field.x,
        y: msg.magnetic_field.y,
        z: msg.magnetic_field.z,
        abs: Math.sqrt(msg.magnetic_field.x ** 2 + msg.magnetic_field.y ** 2 + msg.magnetic_field.z ** 2)
      });
    };
    magnetometerTopic.subscribe(cb);
    return () => magnetometerTopic.unsubscribe(cb);
  }, [rerenderCount]);

  function requestMagnetometerMeasurement() {
    if (magnetometerResetService) {
      magnetometerResetService.callService({}, () => {});
    }
  }

  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const greenBg = style.getPropertyValue('--green-background');
  const blueBg = style.getPropertyValue('--blue-background');
  const magentaBg = style.getPropertyValue('--magenta-background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <>
      <div className={styles['science-row']}>
        <Label color={redBg}>X</Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {magField ? magField.x.toFixed(2) : '---'}
        </Label>
        <Label color={greenBg}>Y</Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {magField ? magField.y.toFixed(2) : '---'}
        </Label>
        <Label color={blueBg}>Z</Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {magField ? magField.z.toFixed(2) : '---'}
        </Label>
      </div>
      <div className={styles['science-row']}>
        <Label color={magentaBg}>
          <FontAwesomeIcon icon={faRuler} />
        </Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {magField ? magField.abs.toFixed(2) : '---'}
        </Label>
      </div>
      <div className={styles['science-row']}>
        <Button
          className={styles['science-row-item']}
          tooltip='Request new magnetometer measurement'
          onClick={requestMagnetometerMeasurement}
        >
          <FontAwesomeIcon icon={faArrowRotateRight} />
          &nbsp;&nbsp;<span style={{ marginTop: '2px' }}>Refresh</span>
        </Button>
      </div>
    </>
  );
}

function DrillContainer() {
  const bInputRef = useRef<Input>(null);
  const cInputRef = useRef<Input>(null);
  const bSkipBlurRef = useRef(false);
  const cSkipBlurRef = useRef(false);
  const [bValue, setBValue] = useState(0);
  const [cValue, setCValue] = useState(0);
  const [autonomyState, setAutonomyState] = useState(0);
  const [telemetry, setTelemetry] = useState<DrillTelemetry | null>(null);
  const [lastTelemetryAt, setLastTelemetryAt] = useState<number | null>(null);
  const [secondsSinceTelemetry, setSecondsSinceTelemetry] = useState<number | null>(null);
  const [rerenderCount, setRerenderCount] = useState(0);

  useEffect(() => {
    const updateScience = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('science-subscribed', updateScience);
    return () => {
      window.removeEventListener('science-subscribed', updateScience);
    };
  }, []);

  useEffect(() => {
    setTelemetry(null);
    setLastTelemetryAt(null);
    setSecondsSinceTelemetry(null);

    console.log(drillTelemetryTopic);
    if (!drillTelemetryTopic) {
      console.log('xc');
      return;
    }

    const cb = (msg: DrillTelemetry) => {
      setTelemetry(msg);
      setLastTelemetryAt(Date.now());
      setSecondsSinceTelemetry(0);
    };

    drillTelemetryTopic.subscribe(cb);
    return () => drillTelemetryTopic.unsubscribe(cb);
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

  const publishBridge = (topic: any, value: number) => {
    topic?.publish({ data: normalizeInteger(value, -100, 100, 0) });
  };

  const commitBridgeB = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeInteger(rawValue ?? bInputRef.current?.getValue(), -100, 100, bValue);
    bSkipBlurRef.current = skipBlur;
    setBValue(nextValue);
    bInputRef.current?.setValue(nextValue);
    publishBridge(drillBTopic, nextValue);
  };

  const commitBridgeC = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeInteger(rawValue ?? cInputRef.current?.getValue(), -100, 100, cValue);
    cSkipBlurRef.current = skipBlur;
    setCValue(nextValue);
    cInputRef.current?.setValue(nextValue);
    publishBridge(drillCTopic, nextValue);
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
    publishBridge(drillBTopic, 0);
  };

  const stopBridgeC = () => {
    cSkipBlurRef.current = false;
    setCValue(0);
    cInputRef.current?.setValue(0);
    publishBridge(drillCTopic, 0);
  };

  const publishAutonomy = (rawValue: number) => {
    const value = normalizeInteger(rawValue, 0, 2, 0);
    setAutonomyState(value);
    drillAutonomyTopic?.publish({ data: value });
  };

  const formatNumber = (value: number | undefined, digits = 1) => {
    return value !== undefined ? value.toFixed(digits) : '---';
  };

  const formatBool = (value: boolean | undefined) => {
    if (value === undefined) return '---';
    return value ? 'Yes' : 'No';
  };

  const formatHex = (value: number | undefined, width = 2) => {
    return value !== undefined ? `0x${value.toString(16).toUpperCase().padStart(width, '0')}` : '---';
  };

  return (
    <div className={styles['science-drill-grid']}>
      <div>
        <div className={styles['science-field-header']}>Bridge B</div>
        <div className={styles['science-row']}>
          <Button className={styles['science-row-item']} tooltip='Move rack up' onClick={() => setBridgeBDirection(true)}>
            <FontAwesomeIcon icon={faArrowUp} />
            &nbsp;&nbsp;Up
          </Button>
          <Button className={styles['science-row-item']} tooltip='Move rack down' onClick={() => setBridgeBDirection(false)}>
            <FontAwesomeIcon icon={faArrowDown} />
            &nbsp;&nbsp;Down
          </Button>
        </div>
        <div className={styles['science-row']}>
          <Button className={styles['science-step-button']} tooltip='Decrease value by 1' onClick={() => commitBridgeB(bValue - 1)}>
            <FontAwesomeIcon icon={faMinus} />
          </Button>
          <Input
            ref={bInputRef}
            type='float'
            className={styles['science-input']}
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
          <Button className={styles['science-step-button']} tooltip='Increase value by 1' onClick={() => commitBridgeB(bValue + 1)}>
            <FontAwesomeIcon icon={faPlus} />
          </Button>
        </div>
        <div className={styles['science-row']}>
          <Button className={styles['science-row-item']} tooltip='Send drill command' onClick={() => commitBridgeB()}>
            <FontAwesomeIcon icon={faPaperPlane} />
            &nbsp;&nbsp;Send
          </Button>
          <Button className={styles['science-row-item'] + ' ' + styles['colored-button'] + ' red'} tooltip='Stop drill command' onClick={stopBridgeB}>
            <FontAwesomeIcon icon={faStop} />
            &nbsp;&nbsp;Stop
          </Button>
        </div>

        <div className={styles['science-field-header']}>Bridge C</div>
        <div className={styles['science-row']}>
          <Button className={styles['science-row-item']} tooltip='Rotate drill right' onClick={() => setBridgeCDirection(true)}>
            <FontAwesomeIcon icon={faArrowRight} />
            &nbsp;&nbsp;Right
          </Button>
          <Button className={styles['science-row-item']} tooltip='Rotate drill left' onClick={() => setBridgeCDirection(false)}>
            <FontAwesomeIcon icon={faArrowLeft} />
            &nbsp;&nbsp;Left
          </Button>
        </div>
        <div className={styles['science-row']}>
          <Button className={styles['science-step-button']} tooltip='Decrease value by 1' onClick={() => commitBridgeC(cValue - 1)}>
            <FontAwesomeIcon icon={faMinus} />
          </Button>
          <Input
            ref={cInputRef}
            type='float'
            className={styles['science-input']}
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
          <Button className={styles['science-step-button']} tooltip='Increase value by 1' onClick={() => commitBridgeC(cValue + 1)}>
            <FontAwesomeIcon icon={faPlus} />
          </Button>
        </div>
        <div className={styles['science-row']}>
          <Button className={styles['science-row-item']} tooltip='Send drill command' onClick={() => commitBridgeC()}>
            <FontAwesomeIcon icon={faPaperPlane} />
            &nbsp;&nbsp;Send
          </Button>
          <Button className={styles['science-row-item'] + ' ' + styles['colored-button'] + ' red'} tooltip='Stop drill command' onClick={stopBridgeC}>
            <FontAwesomeIcon icon={faStop} />
            &nbsp;&nbsp;Stop
          </Button>
        </div>

        <div className={styles['science-field-header']}>Autonomy</div>
        <div className={styles['science-row']}>
          {autonomyStates.map((state) => (
            <Button
              key={state.value}
              className={`${styles['science-row-item']} ${
                autonomyState === state.value ? styles['science-toggle-active'] : styles['science-toggle-inactive']
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
      <div>
        <div className={styles['science-field-header']}>Telemetry</div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Depth</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={`${formatNumber(telemetry?.depth_mm)} mm`} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Rack I</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={`${formatNumber(telemetry?.rack_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Drill I</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={`${formatNumber(telemetry?.drill_current, 2)} A`} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Flags</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={formatHex(telemetry?.flags)} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Upper</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={formatBool(telemetry?.upper_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Lower</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={formatBool(telemetry?.lower_limit_pressed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Active</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={formatBool(telemetry?.autonomy_active)} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>Homed</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={formatBool(telemetry?.homed)} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-row']}>
          <Label color='var(--dark-active)' className={styles['science-telemetry-label']}>State</Label>
          <div className={styles['science-disabled-input'] + ' ' + styles['science-selectable']}>
            <input value={String(telemetry?.autonomy_state ?? '---')} disabled readOnly />
          </div>
        </div>
        <div className={styles['science-received-text']}>
          {secondsSinceTelemetry === null ? 'Received ---' : `Received ${secondsSinceTelemetry} s ago`}
        </div>
      </div>
    </div>
  );
}
