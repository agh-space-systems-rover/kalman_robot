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
  faBan
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';
import { Topic, Service } from 'roslib';

import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Label from '../components/label';

const STORAGE_OPTIONS = [
  { name: 'Sand Storage', value: 'sand', icon: faFlask },
  { name: 'Rock Storage', value: 'rock', icon: faBox },
  { name: 'pH Probe', value: 'ph', icon: faDroplet }
];

// Global ROS clients
let sandWeightTopic: any;
let rockWeightTopic: any;
let phValueTopic: any;
let sandWeightService: any;
let rockWeightService: any;
let sandOpenService: any;
let sandCloseService: any;
let rockOpenService: any;
let rockCloseService: any;
let phValueService: any;

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

  window.dispatchEvent(new Event('science-subscribed'));
});

type SciencePanelProps = {
  props: {
    selectedStorage: string;
    sandTareHistory?: number[];
    rockTareHistory?: number[];
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

  const [selectedStorage, setSelectedStorage] = useState(props.selectedStorage);
  const [rerenderCount, setRerenderCount] = useState(0);

  const getTareHistory = () => {
    return selectedStorage === 'sand' ? props.sandTareHistory : props.rockTareHistory;
  };

  const setTareHistory = (history: number[]) => {
    if (selectedStorage === 'sand') {
      props.sandTareHistory = history;
    } else {
      props.rockTareHistory = history;
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

        {selectedStorage === 'ph' && <PHProbe />}
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
        <Label color={darkBg} className={styles['science-row-item']}>
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

  // Service call helper
  function requestPhMeasurement() {
    if (phValueService) {
      phValueService.callService({}, () => {});
    }
  }

  const style = getComputedStyle(document.body);
  const greenBg = style.getPropertyValue('--green-background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <div className={styles['science-row']}>
      <Label color={greenBg}>
        <FontAwesomeIcon icon={faDroplet} />
      </Label>
      <Label color={darkBg} className={styles['science-row-item']}>
        {phValue !== null ? `${phValue.toFixed(2)} pH` : '---'}
      </Label>
      <Button tooltip='Request new pH measurement' onClick={requestPhMeasurement}>
        <FontAwesomeIcon icon={faArrowRotateRight} />
      </Button>
    </div>
  );
}
