import styles from './science.module.css';

import { ros } from '../common/ros';
import {
  faWeightHanging,
  faFlask,
  faBoxOpen,
  faBox,
  faArrowRotateRight,
  faDroplet,
  faTrash,
  faList,
  faBan,
  faMagnet,
  faArrowDown,
  faArrowUp,
  faStop,
  faDiagramProject,
  faRuler,
  faRobot,
  faOilWell,
  faPlay
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState, useRef } from 'react';
import { Topic, Service } from 'roslib';

import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Label from '../components/label';

const STORAGE_OPTIONS = [
  { name: 'Sand Storage', value: 'sand', icon: faFlask },
  { name: 'Rock Storage', value: 'rock', icon: faBox },
  { name: 'pH Probe', value: 'ph', icon: faDroplet },
  { name: 'Magnetometer', value: 'magneto', icon: faMagnet },
  { name: 'Drill', value: 'drill', icon: faOilWell },
];
// Drill ROS clients
let drillWeightTopic: any;
let drillWeightService: any;
let drillAutoStartService: any;
let drillAutoStopService: any;

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

  // Drill topics/services
  drillWeightTopic = new Topic({
    ros,
    name: '/science/drill/weight',
    messageType: 'std_msgs/Float32'
  });
  drillWeightService = new Service({
    ros,
    name: '/science/drill/weight/req',
    serviceType: 'std_srvs/Trigger'
  });
  drillAutoStartService = new Service({
    ros,
    name: '/science/drill/auto/start',
    serviceType: 'std_srvs/Trigger'
  });
  drillAutoStopService = new Service({
    ros,
    name: '/science/drill/auto/stop',
    serviceType: 'std_srvs/Trigger'
  });

  window.dispatchEvent(new Event('science-subscribed'));
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

type DrillContainerProps = {
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
    <div className={styles['science']}>
      <div className={styles['science-rows']}>
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

        {selectedStorage === 'drill' && (
          <DrillContainer
            tareHistory={getTareHistory()}
            onTareHistoryChange={setTareHistory}
          />
        )}

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
              if (window.confirm('Are you sure you want to clear all tare history?')) {
                clearTareHistory();
              }
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
  const bgColor  = style.getPropertyValue('--background');
  const activeColor = style.getPropertyValue('--active');

  return <>
    <div className={styles['science-row']}>
      <Label color={blueBg}>
        <FontAwesomeIcon icon={faDroplet} />
      </Label>
      <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
        {phValue !== null ? `${phValue.toFixed(2)} pH` : '---'}
      </Label>
    </div>
    <div className={styles['science-row']}>
      <Button className={styles['science-row-item']} tooltip='Request new pH measurement' onClick={requestPhMeasurement}>
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
          &nbsp;&nbsp;
          Movement Speed
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
          console.log(saturation);
          const style = isActive && speed !== 0
            ? { filter: `saturate(${saturation})` }
            : {};

          return (
            <Label
              key={speed}
              color={color}
              className={styles['science-row-item']}
              style={style}
            >
              {null}
            </Label>
          );
        })}
      </div>
      <div className={styles['science-row']}>
        <Button
          className={styles['science-row-item']}
          tooltip="Decrease movement speed"
          onClick={() => setMoveSpeed((speed) => Math.max(-MAX_MOVE_SPEED, speed - 1))}
        >
          <FontAwesomeIcon icon={faArrowDown} />
          &nbsp;&nbsp;
          <span style={{ marginTop: '2px' }}>Down</span>
        </Button>
        <Button
          className={styles['science-row-item']}
          tooltip="Stop movement"
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
          tooltip="Increase movement speed"
          onClick={() => setMoveSpeed((speed) => Math.min(MAX_MOVE_SPEED, speed + 1))}
        >
          <FontAwesomeIcon icon={faArrowUp} />
          &nbsp;&nbsp;
          <span style={{ marginTop: '2px' }}>Up</span>
        </Button>
      </div>
    </div></div>
    </>
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
        abs: Math.sqrt(msg.magnetic_field.x**2 + msg.magnetic_field.y**2 + msg.magnetic_field.z**2), 
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

  return <>
    <div className={styles['science-row']}>
      <Label  color={redBg}>X</Label>
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
      <Button className={styles['science-row-item']} tooltip='Request new magnetometer measurement' onClick={requestMagnetometerMeasurement}>
        <FontAwesomeIcon icon={faArrowRotateRight} />
        &nbsp;&nbsp;<span style={{ marginTop: '2px' }}>Refresh</span>
      </Button>
    </div>
  </>;
}

function DrillContainer({ tareHistory, onTareHistoryChange }: DrillContainerProps) {
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

  // Subscribe to drill weight topic
  useEffect(() => {
    setWeight(null);
    if (!drillWeightTopic) return;
    const cb = (msg: { data: number }) => {
      setWeight(msg.data);
    };
    drillWeightTopic.subscribe(cb);
    return () => drillWeightTopic.unsubscribe(cb);
  }, [rerenderCount]);

  // Service call helpers
  function callDrillWeightService() {
    if (drillWeightService) {
      drillWeightService.callService({}, () => {});
    }
  }

  function handleTare() {
    if (weight !== null) {
      const currentTareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
      const tareValue = weight - currentTareOffset;
      const newHistory = [...tareHistory, tareValue];
      onTareHistoryChange(newHistory);
    }
  }

  function clearTareHistory() {
    onTareHistoryChange([]);
  }

  function handleStartAutonomy() {
    if (drillAutoStartService) {
      drillAutoStartService.callService({}, () => {});
    }
  }

  function handleStopAutonomy() {
    if (drillAutoStopService) {
      drillAutoStopService.callService({}, () => {});
    }
  }

  const tareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
  const displayWeight = weight !== null ? weight - tareOffset : null;

  const style = getComputedStyle(document.body);
  const magentaBg = style.getPropertyValue('--magenta-background');
  const greenBg = style.getPropertyValue('--green-background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <>
      <div className={styles['science-row']}>
        <Label color={magentaBg}>
          <FontAwesomeIcon icon={faRobot} />
        </Label>
        <Button
          className={styles['science-row-item']}
          tooltip='Start drill autonomy'
          onClick={handleStartAutonomy}
        >
          <FontAwesomeIcon icon={faPlay} />
          &nbsp;&nbsp;Start
        </Button>
        <Button
          className={styles['science-row-item']}
          tooltip='Stop drill autonomy'
          onClick={handleStopAutonomy}
        >
          <FontAwesomeIcon icon={faStop} />
          &nbsp;&nbsp;Stop
        </Button>
      </div>

      <div className={styles['science-row']}>
        <Label color={greenBg}>
          <FontAwesomeIcon icon={faWeightHanging} />
        </Label>
        <Label color={darkBg} className={styles['science-row-item'] + ' ' + styles['science-selectable']}>
          {displayWeight !== null ? `${displayWeight.toFixed(2)} g` : '---'}
        </Label>
        <Button tooltip='Refresh weight measurement' onClick={callDrillWeightService}>
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
              if (window.confirm('Are you sure you want to clear all tare history?')) {
                clearTareHistory();
              }
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
