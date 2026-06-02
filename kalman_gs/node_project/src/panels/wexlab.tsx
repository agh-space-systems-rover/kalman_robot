import styles from './wexlab.module.css';

import { modalRef } from '../common/refs';
import { ros } from '../common/ros';
import { ColorRGBA, WExLabHeaterCfg, WExLabLedAll, WExLabLedSingle, WExLabTemperature } from '../common/ros-interfaces';
import {
  faBoxArchive,
  faFire,
  faDroplet,
  faBoxOpen,
  faBox,
  faPaperPlane,
  faMinus,
  faPlus,
  faPlay,
  faStop,
  faWeightHanging,
  faArrowRotateRight,
  faScaleBalanced,
  faList,
  faBan,
  faTrash,
  faLightbulb,
  faHashtag,
  faTemperatureThreeQuarters
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { RefObject, useEffect, useRef, useState } from 'react';
import { HexColorPicker } from 'react-colorful';
import { Topic } from 'roslib';

import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';

const WEXLAB_OPTIONS = [
  { name: 'Lenka Box', value: 'lenka', icon: faBoxArchive },
  { name: 'Pump', value: 'pump', icon: faDroplet },
  { name: 'Heater', value: 'heater', icon: faFire },
  { name: 'Weight', value: 'weight', icon: faScaleBalanced },
  { name: 'Temperature', value: 'temperature', icon: faTemperatureThreeQuarters },
  { name: 'Servo', value: 'servo', icon: faBoxOpen },
  { name: 'LED', value: 'led', icon: faLightbulb }
] as const;

type WexlabSection = (typeof WEXLAB_OPTIONS)[number]['value'];

let lenkaTopic: Topic<{ data: number }> | undefined;
let pumpTopic: Topic<{ data: number }> | undefined;
let heaterOnOffTopic: Topic<{ data: boolean }> | undefined;
let heaterCfgTopic: Topic<WExLabHeaterCfg> | undefined;
let weightReqTopic: Topic<Record<string, never>> | undefined;
let weightTareTopic: Topic<Record<string, never>> | undefined;
let weightResTopic: Topic<{ data: number }> | undefined;
let temperatureReqTopic: Topic<{ data: number }> | undefined;
let temperatureResTopic: Topic<WExLabTemperature> | undefined;
let servoOpenTopic: Topic<{ data: number }> | undefined;
let servoOnOffTopic: Topic<{ data: boolean }> | undefined;
let ledAllTopic: Topic<WExLabLedAll> | undefined;
let ledSingleTopic: Topic<WExLabLedSingle> | undefined;

window.addEventListener('ros-connect', () => {
  lenkaTopic = new Topic({
    ros,
    name: '/science/front_sand_storage/cmd_open_close',
    messageType: 'std_msgs/Float32'
  });

  pumpTopic = new Topic({
    ros,
    name: '/wexlab/pump/rate_cmd',
    messageType: 'std_msgs/Float32'
  });

  heaterOnOffTopic = new Topic({
    ros,
    name: '/wexlab/heater/on_off',
    messageType: 'std_msgs/Bool'
  });

  heaterCfgTopic = new Topic({
    ros,
    name: '/wexlab/heater/cfg',
    messageType: 'kalman_interfaces/WExLabHeaterCfg'
  });

  weightReqTopic = new Topic({
    ros,
    name: '/wexlab/weight/req',
    messageType: 'std_msgs/Empty'
  });

  weightTareTopic = new Topic({
    ros,
    name: '/wexlab/weight/tare',
    messageType: 'std_msgs/Empty'
  });

  weightResTopic = new Topic({
    ros,
    name: '/wexlab/weight/res',
    messageType: 'std_msgs/Float32'
  });

  temperatureReqTopic = new Topic({
    ros,
    name: '/wexlab/temperature/req',
    messageType: 'std_msgs/UInt8'
  });

  temperatureResTopic = new Topic({
    ros,
    name: '/wexlab/temperature/res',
    messageType: 'kalman_interfaces/WExLabTemperature'
  });

  servoOpenTopic = new Topic({
    ros,
    name: '/wexlab/lid/open_cmd',
    messageType: 'std_msgs/Float32'
  });

  servoOnOffTopic = new Topic({
    ros,
    name: '/wexlab/lid/on_off',
    messageType: 'std_msgs/Bool'
  });

  ledAllTopic = new Topic({
    ros,
    name: '/wexlab/led/all',
    messageType: 'kalman_interfaces/WExLabLedAll'
  });

  ledSingleTopic = new Topic({
    ros,
    name: '/wexlab/led/single',
    messageType: 'kalman_interfaces/WExLabLedSingle'
  });

  window.dispatchEvent(new CustomEvent('wexlab-subscribed'));
});

type WexlabPanelProps = {
  props: {
    selectedSection?: WexlabSection;
    lenkaBoxValue?: number;
    pumpValue?: number;
    heaterThermostatMin?: number;
    heaterThermostatMax?: number;
    heaterPowerMain?: number;
    heaterPowerLid?: number;
    weightTareHistory?: number[];
    servoValue?: number;
    ledTarget?: 'all' | number;
    ledColor?: string;
  };
};

type ValueEditorProps = {
  min: number;
  max: number;
  step?: number;
  placeholder?: string;
  value?: number;
  onValueChange: (value: number) => void;
  onSend: (value: number) => void;
  onReset?: () => void;
};

type HeaterCfgKey = 'heaterThermostatMin' | 'heaterThermostatMax' | 'heaterPowerMain' | 'heaterPowerLid';

type HeaterField = {
  key: HeaterCfgKey;
  label: string;
  placeholder: string;
};

const HEATER_FIELDS: HeaterField[] = [
  { key: 'heaterThermostatMin', label: 'Low Temp', placeholder: 'Thermostat Min' },
  { key: 'heaterThermostatMax', label: 'High Temp', placeholder: 'Thermostat Max' },
  { key: 'heaterPowerMain', label: 'Main PWM', placeholder: 'Power Main' },
  { key: 'heaterPowerLid', label: 'Lid PWM', placeholder: 'Power Lid' }
];

function clamp(value: number, min: number, max: number) {
  return Math.min(Math.max(value, min), max);
}

function normalizeNumber(value: unknown, min: number, max: number, fallback: number) {
  const parsed = typeof value === 'number' ? value : parseFloat(String(value ?? ''));
  if (Number.isNaN(parsed)) return fallback;
  return clamp(parsed, min, max);
}

export default function Wexlab({ props }: WexlabPanelProps) {
  if (props.selectedSection === undefined) props.selectedSection = 'lenka';
  if (props.lenkaBoxValue === undefined) props.lenkaBoxValue = 0;
  if (props.pumpValue === undefined) props.pumpValue = 0;
  if (props.heaterThermostatMin === undefined) props.heaterThermostatMin = 0;
  if (props.heaterThermostatMax === undefined) props.heaterThermostatMax = 0;
  if (props.heaterPowerMain === undefined) props.heaterPowerMain = 0;
  if (props.heaterPowerLid === undefined) props.heaterPowerLid = 0;
  if (props.weightTareHistory === undefined) props.weightTareHistory = [];
  if (props.servoValue === undefined) props.servoValue = 0;
  if (props.ledTarget === undefined) props.ledTarget = 'all';
  if (props.ledColor === undefined) props.ledColor = '#000080';

  const [selectedSection, setSelectedSection] = useState<WexlabSection>(props.selectedSection);
  const [, setRerenderCount] = useState(0);

  return (
    <div className={styles['wexlab']}>
      <div className={styles['wexlab-rows']}>
        <div className={styles['wexlab-row']}>
          <Dropdown
            className={styles['wexlab-row-item']}
            tooltip='Select WExLab instrument'
            items={WEXLAB_OPTIONS.map((opt) => ({
              icon: opt.icon,
              text: opt.name
            }))}
            defaultItemIndex={WEXLAB_OPTIONS.findIndex((opt) => opt.value === selectedSection)}
            onChange={(i) => {
              const next = WEXLAB_OPTIONS[i].value;
              props.selectedSection = next;
              setSelectedSection(next);
            }}
          />
        </div>

        {selectedSection === 'lenka' && (
          <LenkaBoxEditor
            value={props.lenkaBoxValue}
            onValueChange={(value) => {
              props.lenkaBoxValue = value;
            }}
            onSend={(value) => {
              lenkaTopic?.publish({ data: value });
            }}
          />
        )}

        {selectedSection === 'pump' && (
          <ValueEditor
            min={0}
            max={100}
            placeholder='Pump Rate'
            value={props.pumpValue}
            onValueChange={(value) => {
              props.pumpValue = value;
            }}
            onSend={(value) => {
              pumpTopic?.publish({ data: value });
            }}
            onReset={() => {
              props.pumpValue = 0;
              pumpTopic?.publish({ data: 0 });
            }}
          />
        )}

        {selectedSection === 'heater' && <HeaterPanel props={props} />}
        {selectedSection === 'weight' && (
          <WeightPanel
            tareHistory={props.weightTareHistory}
            onTareHistoryChange={(history) => {
              props.weightTareHistory = history;
              setRerenderCount((count) => count + 1);
            }}
          />
        )}
        {selectedSection === 'temperature' && <TemperaturePanel />}
        {selectedSection === 'servo' && (
          <ServoPanel
            value={props.servoValue}
            onValueChange={(value) => {
              props.servoValue = value;
            }}
          />
        )}
        {selectedSection === 'led' && (
          <LedPanel
            target={props.ledTarget}
            color={props.ledColor}
            onTargetChange={(target) => {
              props.ledTarget = target;
              setRerenderCount((count) => count + 1);
            }}
            onColorChange={(color) => {
              props.ledColor = color;
              setRerenderCount((count) => count + 1);
            }}
          />
        )}
      </div>
    </div>
  );
}

function hexToColorRGB(hexColor: string): ColorRGBA {
  const raw = hexColor.trim().replace(/^#/, '').toUpperCase();

  if (raw.length === 3) {
    const [r3, g3, b3] = raw;
    return {
      r: parseInt(r3 + r3, 16) / 255,
      g: parseInt(g3 + g3, 16) / 255,
      b: parseInt(b3 + b3, 16) / 255
    };
  }

  if (raw.length === 6) {
    return {
      r: parseInt(raw.slice(0, 2), 16) / 255,
      g: parseInt(raw.slice(2, 4), 16) / 255,
      b: parseInt(raw.slice(4, 6), 16) / 255
    };
  }

  throw new Error(`Hex color code ("${hexColor}") is invalid.`);
}

function hexToGrayscaleReversed(hexColor: string): string {
  const raw = hexColor.trim().replace(/^#/, '').toUpperCase();

  let r = 0;
  let g = 0;
  let b = 0;

  if (raw.length === 3) {
    const [r3, g3, b3] = raw;
    r = parseInt(r3 + r3, 16);
    g = parseInt(g3 + g3, 16);
    b = parseInt(b3 + b3, 16);
  } else if (raw.length === 6) {
    r = parseInt(raw.slice(0, 2), 16);
    g = parseInt(raw.slice(2, 4), 16);
    b = parseInt(raw.slice(4, 6), 16);
  }

  const brightness = 0.299 * r + 0.587 * g + 0.114 * b;
  return brightness > 128 ? '#111111' : '#EEEEEE';
}

function ValueEditor({ min, max, step = 1, placeholder, value = 0, onValueChange, onSend, onReset }: ValueEditorProps) {
  const [inputValue, setInputValue] = useState(value);
  const inputRef = useRef<Input>(null);
  const skipBlurRef = useRef(false);

  useEffect(() => {
    const normalized = normalizeNumber(value, min, max, 0);
    setInputValue(normalized);
    inputRef.current?.setValue(normalized);
  }, [max, min, value]);

  const commitValue = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeNumber(rawValue ?? inputRef.current?.getValue(), min, max, inputValue);
    skipBlurRef.current = skipBlur;
    setInputValue(nextValue);
    onValueChange(nextValue);
    inputRef.current?.setValue(nextValue);
    onSend(nextValue);
  };

  const shiftValue = (delta: number) => {
    commitValue(inputValue + delta);
  };

  const resetValue = () => {
    setInputValue(0);
    onValueChange(0);
    inputRef.current?.setValue(0);
    if (onReset) {
      onReset();
      return;
    }
    onSend(0);
  };

  return (
    <>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-step-button']} tooltip={`Decrease value by ${step}`} onClick={() => shiftValue(-step)}>
          <FontAwesomeIcon icon={faMinus} />
        </Button>
        <Input
          ref={inputRef}
          type='float'
          className={styles['wexlab-input']}
          placeholder={placeholder}
          defaultValue={String(inputValue)}
          onChange={(text) => {
            const nextValue = normalizeNumber(text, min, max, inputValue);
            setInputValue(nextValue);
            onValueChange(nextValue);
          }}
          onSubmit={(text) => {
            commitValue(text, true);
          }}
          onBlur={() => {
            if (skipBlurRef.current) {
              skipBlurRef.current = false;
              return;
            }
            commitValue();
          }}
        />
        <Button className={styles['wexlab-step-button']} tooltip={`Increase value by ${step}`} onClick={() => shiftValue(step)}>
          <FontAwesomeIcon icon={faPlus} />
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Send command' onClick={() => commitValue()}>
          <FontAwesomeIcon icon={faPaperPlane} />
          &nbsp;&nbsp;Send
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Reset value to zero' onClick={resetValue}>
          <FontAwesomeIcon icon={faArrowRotateRight} />
          &nbsp;&nbsp;Reset
        </Button>
      </div>
    </>
  );
}

function LenkaBoxEditor({
  value = 0,
  onValueChange,
  onSend
}: {
  value?: number;
  onValueChange: (value: number) => void;
  onSend: (value: number) => void;
}) {
  const [inputValue, setInputValue] = useState(value);
  const inputRef = useRef<Input>(null);
  const skipBlurRef = useRef(false);

  useEffect(() => {
    const normalized = normalizeNumber(value, -50, 50, 0);
    setInputValue(normalized);
    inputRef.current?.setValue(normalized);
  }, [value]);

  const commitValue = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeNumber(rawValue ?? inputRef.current?.getValue(), -100, 100, inputValue);
    skipBlurRef.current = skipBlur;
    setInputValue(nextValue);
    onValueChange(nextValue);
    inputRef.current?.setValue(nextValue);
    onSend(nextValue);
  };

  const stopValue = () => {
    skipBlurRef.current = false;
    onSend(0);
  };

  const shiftValue = (delta: number) => {
    commitValue(inputValue + delta);
  };

  const setDirection = (open: boolean) => {
    const baseValue = Math.abs(normalizeNumber(inputRef.current?.getValue(), -100, 100, inputValue));
    const nextValue = open ? baseValue : -baseValue;
    commitValue(nextValue);
  };

  return (
    <>
      <div className={styles['wexlab-row']}>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' green'}
          tooltip='Set open direction'
          onClick={() => setDirection(true)}
        >
          <FontAwesomeIcon icon={faBoxOpen} />
          &nbsp;&nbsp;Open
        </Button>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' red'}
          tooltip='Set close direction'
          onClick={() => setDirection(false)}
        >
          <FontAwesomeIcon icon={faBox} />
          &nbsp;&nbsp;Close
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-step-button']} tooltip='Decrease value by 1' onClick={() => shiftValue(-1)}>
          <FontAwesomeIcon icon={faMinus} />
        </Button>
        <Input
          ref={inputRef}
          type='float'
          className={styles['wexlab-input']}
          placeholder='Lenka Box'
          defaultValue={String(inputValue)}
          onChange={(text) => {
            const nextValue = normalizeNumber(text, -100, 100, inputValue);
            setInputValue(nextValue);
            onValueChange(nextValue);
          }}
          onSubmit={(text) => {
            commitValue(text, true);
          }}
          onBlur={() => {
            if (skipBlurRef.current) {
              skipBlurRef.current = false;
              return;
            }
            commitValue();
          }}
        />
        <Button className={styles['wexlab-step-button']} tooltip='Increase value by 1' onClick={() => shiftValue(1)}>
          <FontAwesomeIcon icon={faPlus} />
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Send command' onClick={() => commitValue()}>
          <FontAwesomeIcon icon={faPaperPlane} />
          &nbsp;&nbsp;Send
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' red'}
          tooltip='Quick stop mechanism'
          onClick={stopValue}
        >
          <FontAwesomeIcon icon={faStop} />
          &nbsp;&nbsp;Stop
        </Button>
      </div>
    </>
  );
}

type NumberInputRowProps = {
  inputRef: RefObject<Input>;
  inputValue: number;
  className?: string;
  placeholder?: string;
  disabled?: boolean;
  onChange: (text: string) => void;
  onSubmit: (text: string) => void;
  onBlur: () => void;
  onDecrease: () => void;
  onIncrease: () => void;
};

function NumberInputRow({
  inputRef,
  inputValue,
  className,
  placeholder,
  disabled = false,
  onChange,
  onSubmit,
  onBlur,
  onDecrease,
  onIncrease
}: NumberInputRowProps) {
  return (
    <div className={styles['wexlab-row']}>
      <Button className={styles['wexlab-step-button']} tooltip='Decrease value by 1' onClick={onDecrease} disabled={disabled}>
        <FontAwesomeIcon icon={faMinus} />
      </Button>
      <Input
        ref={inputRef}
        type='float'
        className={className}
        placeholder={placeholder}
        defaultValue={String(inputValue)}
        disabled={disabled}
        onChange={onChange}
        onSubmit={onSubmit}
        onBlur={onBlur}
      />
      <Button className={styles['wexlab-step-button']} tooltip='Increase value by 1' onClick={onIncrease} disabled={disabled}>
        <FontAwesomeIcon icon={faPlus} />
      </Button>
    </div>
  );
}

function HeaterPanel({ props }: WexlabPanelProps) {
  const thermostatMinRef = useRef<Input>(null);
  const thermostatMaxRef = useRef<Input>(null);
  const powerMainRef = useRef<Input>(null);
  const powerLidRef = useRef<Input>(null);
  const skipBlurRef = useRef<Record<HeaterCfgKey, boolean>>({
    heaterThermostatMin: false,
    heaterThermostatMax: false,
    heaterPowerMain: false,
    heaterPowerLid: false
  });

  useEffect(() => {
    const update = () => {};
    window.addEventListener('wexlab-subscribed', update);
    return () => {
      window.removeEventListener('wexlab-subscribed', update);
    };
  }, []);

  const publishOnOff = (value: boolean) => {
    heaterOnOffTopic?.publish({ data: value });
  };

  const publishCfg = () => {
    const cfg: WExLabHeaterCfg = {
      thermostat_min: normalizeNumber(props.heaterThermostatMin, 0, 100, 0),
      thermostat_max: normalizeNumber(props.heaterThermostatMax, 0, 100, 0),
      power_main: normalizeNumber(props.heaterPowerMain, 0, 100, 0),
      power_lid: normalizeNumber(props.heaterPowerLid, 0, 100, 0)
    };

    props.heaterThermostatMin = cfg.thermostat_min;
    props.heaterThermostatMax = cfg.thermostat_max;
    props.heaterPowerMain = cfg.power_main;
    props.heaterPowerLid = cfg.power_lid;

    heaterCfgTopic?.publish(cfg);
  };

  const setFieldValue = (key: HeaterCfgKey, value: number, commit = false, skipBlur = false) => {
    props[key] = value;
    if (commit) {
      skipBlurRef.current[key] = skipBlur;
      publishCfg();
    }
  };

  const resetHeater = () => {
    for (const field of HEATER_FIELDS) {
      props[field.key] = 0;
      skipBlurRef.current[field.key] = false;
      fieldRefs[field.key].current?.setValue(0);
    }
    publishOnOff(false);
    publishCfg();
  };

  const fieldRefs: Record<HeaterCfgKey, RefObject<Input>> = {
    heaterThermostatMin: thermostatMinRef,
    heaterThermostatMax: thermostatMaxRef,
    heaterPowerMain: powerMainRef,
    heaterPowerLid: powerLidRef
  };

  return (
    <>
      <div className={styles['wexlab-row']}>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' green'}
          tooltip='Turn heater on'
          onClick={() => publishOnOff(true)}
        >
          <FontAwesomeIcon icon={faPlay} />
          &nbsp;&nbsp;On
        </Button>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' red'}
          tooltip='Turn heater off'
          onClick={() => publishOnOff(false)}
        >
          <FontAwesomeIcon icon={faStop} />
          &nbsp;&nbsp;Off
        </Button>
      </div>

      {HEATER_FIELDS.map((field) => (
        <div key={field.key}>
          <div className={styles['wexlab-field-header']}>{field.label}</div>
          <NumberInputRow
            inputRef={fieldRefs[field.key]}
            inputValue={normalizeNumber(props[field.key], 0, 100, 0)}
            className={styles['wexlab-input']}
            placeholder={field.placeholder}
            onChange={(text) => {
              setFieldValue(field.key, normalizeNumber(text, 0, 100, props[field.key] ?? 0));
            }}
            onSubmit={(text) => {
              const nextValue = normalizeNumber(text, 0, 100, props[field.key] ?? 0);
              fieldRefs[field.key].current?.setValue(nextValue);
              setFieldValue(field.key, nextValue, true, true);
            }}
            onBlur={() => {
              if (skipBlurRef.current[field.key]) {
                skipBlurRef.current[field.key] = false;
                return;
              }
              const nextValue = normalizeNumber(fieldRefs[field.key].current?.getValue(), 0, 100, props[field.key] ?? 0);
              fieldRefs[field.key].current?.setValue(nextValue);
              setFieldValue(field.key, nextValue, true);
            }}
            onDecrease={() => {
              const nextValue = normalizeNumber((props[field.key] ?? 0) - 1, 0, 100, 0);
              fieldRefs[field.key].current?.setValue(nextValue);
              setFieldValue(field.key, nextValue, true);
            }}
            onIncrease={() => {
              const nextValue = normalizeNumber((props[field.key] ?? 0) + 1, 0, 100, 0);
              fieldRefs[field.key].current?.setValue(nextValue);
              setFieldValue(field.key, nextValue, true);
            }}
          />
        </div>
      ))}

      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Send heater configuration' onClick={publishCfg}>
          <FontAwesomeIcon icon={faPaperPlane} />
          &nbsp;&nbsp;Send
        </Button>
      </div>
      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Reset heater configuration to zero' onClick={resetHeater}>
          <FontAwesomeIcon icon={faArrowRotateRight} />
          &nbsp;&nbsp;Reset
        </Button>
      </div>
    </>
  );
}

function ServoPanel({
  value = 0,
  onValueChange
}: {
  value?: number;
  onValueChange: (value: number) => void;
}) {
  return (
    <>
      <div className={styles['wexlab-row']}>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' green'}
          tooltip='Turn servo on'
          onClick={() => servoOnOffTopic?.publish({ data: true })}
        >
          <FontAwesomeIcon icon={faPlay} />
          &nbsp;&nbsp;On
        </Button>
        <Button
          className={styles['wexlab-row-item'] + ' ' + styles['colored-button'] + ' red'}
          tooltip='Turn servo off'
          onClick={() => servoOnOffTopic?.publish({ data: false })}
        >
          <FontAwesomeIcon icon={faStop} />
          &nbsp;&nbsp;Off
        </Button>
      </div>

      <ValueEditor
        min={0}
        max={100}
        placeholder='Servo Open %'
        value={value}
        onValueChange={onValueChange}
        onSend={(nextValue) => {
          servoOpenTopic?.publish({ data: nextValue });
        }}
      />
    </>
  );
}

function LedPanel({
  target,
  color,
  onTargetChange,
  onColorChange
}: {
  target: 'all' | number;
  color: string;
  onTargetChange: (target: 'all' | number) => void;
  onColorChange: (color: string) => void;
}) {
  const targetInputRef = useRef<Input>(null);
  const colorInputRef = useRef<Input>(null);
  const skipTargetBlurRef = useRef(false);
  const [targetInputValue, setTargetInputValue] = useState(target === 'all' ? 0 : normalizeNumber(target, 0, 59, 0));
  const [colorInputValue, setColorInputValue] = useState(color.replace('#', ''));

  useEffect(() => {
    const normalizedTarget = target === 'all' ? 0 : normalizeNumber(target, 0, 59, 0);
    setTargetInputValue(normalizedTarget);
    targetInputRef.current?.setValue(normalizedTarget);
  }, [target]);

  useEffect(() => {
    colorInputRef.current?.setValue(color.replace('#', ''));
    setColorInputValue(color.replace('#', ''));
  }, [color]);

  const sendLedRequest = () => {
    const colorObject = hexToColorRGB(color);

    if (target === 'all') {
      ledAllTopic?.publish({ color: colorObject });
      return;
    }

    ledSingleTopic?.publish({
      led_id: normalizeNumber(target, 0, 59, 0),
      color: colorObject
    });
  };

  const commitTargetValue = (rawValue?: unknown, skipBlur = false) => {
    const nextValue = normalizeNumber(rawValue ?? targetInputRef.current?.getValue(), 0, 59, targetInputValue);
    skipTargetBlurRef.current = skipBlur;
    setTargetInputValue(nextValue);
    onTargetChange(nextValue);
    targetInputRef.current?.setValue(nextValue);
  };

  const shiftTargetValue = (delta: number) => {
    commitTargetValue(targetInputValue + delta);
  };

  const handleHexColorInput = (value: string) => {
    const hexRegex = /^[A-Fa-f0-9]{0,6}$/;
    const hexColorRegex = /^([A-Fa-f0-9]{3}|[A-Fa-f0-9]{6})$/;

    if (hexRegex.test(value)) {
      setColorInputValue(value);
    } else {
      colorInputRef.current?.setValue(colorInputValue);
    }

    if (hexColorRegex.test(value)) {
      onColorChange('#' + value);
    }
  };

  return (
    <>
      <div className={styles['wexlab-row']}>
        <Button
          className={`${styles['wexlab-row-item']} ${target === 'all' ? styles['wexlab-toggle-active'] : styles['wexlab-toggle-inactive']}`}
          tooltip='Select all LEDs'
          onClick={() => onTargetChange('all')}
        >
          All
        </Button>
        <Button
          className={`${styles['wexlab-row-item']} ${target !== 'all' ? styles['wexlab-toggle-active'] : styles['wexlab-toggle-inactive']}`}
          tooltip='Select single LED'
          onClick={() => onTargetChange(targetInputValue)}
        >
          Single
        </Button>
      </div>

      <NumberInputRow
        inputRef={targetInputRef}
        inputValue={targetInputValue}
        className={styles['wexlab-input']}
        placeholder='LED ID'
        onChange={(text) => {
          const nextValue = normalizeNumber(text, 0, 59, targetInputValue);
          setTargetInputValue(nextValue);
          onTargetChange(nextValue);
        }}
        onSubmit={(text) => {
          commitTargetValue(text, true);
        }}
        onBlur={() => {
          if (skipTargetBlurRef.current) {
            skipTargetBlurRef.current = false;
            return;
          }
          commitTargetValue();
        }}
        onDecrease={() => shiftTargetValue(-1)}
        onIncrease={() => shiftTargetValue(1)}
        disabled={target === 'all'}
      />

      <div className={styles['wexlab-row']}>
        <HexColorPicker
          className={`${styles['wexlab-row-item']} ${styles['wexlab-color-picker']}`}
          color={color}
          onChange={(value) => {
            onColorChange(value);
            colorInputRef.current?.setValue(value.replace('#', ''));
            setColorInputValue(value.replace('#', ''));
          }}
        />
      </div>

      <div className={styles['wexlab-row']}>
        <Label className={styles['wexlab-hex-label']} style={{ backgroundColor: color, color: hexToGrayscaleReversed(color) }}>
          <FontAwesomeIcon icon={faHashtag} />
        </Label>
        <Input
          ref={colorInputRef}
          defaultValue={color.replace('#', '')}
          maxLength={6}
          onChange={handleHexColorInput}
          onBlur={() => colorInputRef.current?.setValue(color.replace('#', ''))}
        />
      </div>

      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Send LED color' onClick={sendLedRequest}>
          <FontAwesomeIcon icon={faPaperPlane} />
          &nbsp;&nbsp;Send
        </Button>
      </div>
    </>
  );
}

function WeightPanel({
  tareHistory,
  onTareHistoryChange
}: {
  tareHistory: number[];
  onTareHistoryChange: (history: number[]) => void;
}) {
  const [weight, setWeight] = useState<number | null>(null);
  const [rerenderCount, setRerenderCount] = useState(0);
  const [isPending, setIsPending] = useState(false);
  const requestTimeoutRef = useRef<number | undefined>(undefined);

  useEffect(() => {
    const update = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('wexlab-subscribed', update);
    return () => {
      window.removeEventListener('wexlab-subscribed', update);
    };
  }, []);

  useEffect(() => {
    setWeight(null);
    if (!weightResTopic) return;

    const cb = (msg: { data: number }) => {
      setWeight(msg.data);
      setIsPending(false);
      if (requestTimeoutRef.current !== undefined) {
        window.clearTimeout(requestTimeoutRef.current);
        requestTimeoutRef.current = undefined;
      }
    };

    weightResTopic.subscribe(cb);
    return () => weightResTopic?.unsubscribe(cb);
  }, [rerenderCount]);

  useEffect(() => {
    return () => {
      if (requestTimeoutRef.current !== undefined) {
        window.clearTimeout(requestTimeoutRef.current);
      }
    };
  }, []);

  const requestWeight = () => {
    setIsPending(true);
    if (requestTimeoutRef.current !== undefined) {
      window.clearTimeout(requestTimeoutRef.current);
    }
    requestTimeoutRef.current = window.setTimeout(() => {
      setIsPending(false);
      requestTimeoutRef.current = undefined;
    }, 3000);
    weightReqTopic?.publish({});
  };

  const tareWeight = () => {
    if (weight !== null) {
      const currentTareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
      const tareValue = weight - currentTareOffset;
      onTareHistoryChange([...tareHistory, tareValue]);
    }
    weightTareTopic?.publish({});
  };

  const clearTareHistory = () => {
    onTareHistoryChange([]);
  };

  const displayWeight = weight !== null ? weight : null;

  const style = getComputedStyle(document.body);
  const greenBg = style.getPropertyValue('--green-background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <>
      <div className={styles['wexlab-row']}>
        <Label color={greenBg}>
          <FontAwesomeIcon icon={faWeightHanging} />
        </Label>
        <Label color={darkBg} className={styles['wexlab-row-item'] + ' ' + styles['wexlab-selectable']}>
          {displayWeight !== null ? `${displayWeight.toFixed(2)} g` : '---'}
        </Label>
        <Button tooltip='Request weight measurement' onClick={requestWeight}>
          <FontAwesomeIcon className={isPending ? styles['wexlab-loading-icon'] : undefined} icon={faArrowRotateRight} />
        </Button>
        <Button tooltip='Tare weight' onClick={tareWeight} disabled={weight === null}>
          <FontAwesomeIcon icon={faScaleBalanced} />
        </Button>
      </div>

      <div className={styles['wexlab-row']}>
        <div className={styles['tare-history']}>
          <div className={styles['wexlab-row']}>
            <Label className={styles['tare-history-header']}>
              <FontAwesomeIcon icon={tareHistory.length > 0 ? faList : faBan} />
              &nbsp; {tareHistory.length > 0 ? '' : 'No'} Tare History
            </Label>
          </div>
          {tareHistory.map((tareValue, index) => (
            <div key={index} className={styles['wexlab-row']}>
              <Label className={styles['tare-entry']}>
                {index + 1}. {tareValue.toFixed(2)} g
              </Label>
            </div>
          ))}
        </div>
      </div>

      {tareHistory.length > 0 && (
        <div className={styles['wexlab-row']}>
          <Button
            className={styles['wexlab-row-item']}
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

type TemperatureStatus = 'idle' | 'ok' | 'error';

type TemperatureReading = {
  id: number;
  label: string;
  value: number | null;
  status: TemperatureStatus;
};

const TEMPERATURE_FIELDS: Array<Pick<TemperatureReading, 'id' | 'label'>> = [
  { id: 0, label: 'Main Heater' },
  { id: 1, label: 'Chamber Temperature' },
  { id: 2, label: 'Auxiliary Measurement' }
];

function TemperaturePanel() {
  const [rerenderCount, setRerenderCount] = useState(0);
  const [readings, setReadings] = useState<TemperatureReading[]>(
    TEMPERATURE_FIELDS.map((field) => ({
      ...field,
      value: null,
      status: 'idle'
    }))
  );
  const [pendingIds, setPendingIds] = useState<number[]>([]);
  const requestTimeoutsRef = useRef<Record<number, number | undefined>>({});

  useEffect(() => {
    const update = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('wexlab-subscribed', update);
    return () => {
      window.removeEventListener('wexlab-subscribed', update);
    };
  }, []);

  useEffect(() => {
    setReadings(
      TEMPERATURE_FIELDS.map((field) => ({
        ...field,
        value: null,
        status: 'idle'
      }))
    );

    if (!temperatureResTopic) return;

    const cb = (msg: WExLabTemperature) => {
      const temperatureId = typeof msg.temperature_id === 'number' ? msg.temperature_id : -1;
      if (!TEMPERATURE_FIELDS.some((field) => field.id === temperatureId)) return;

      setReadings((current) =>
        current.map((reading) =>
          reading.id === temperatureId
            ? {
                ...reading,
                value: typeof msg.temperature === 'number' ? msg.temperature : null,
                status: msg.temperature_error ? 'error' : 'ok'
              }
            : reading
        )
      );
      setPendingIds((current) => current.filter((id) => id !== temperatureId));
      const timeoutId = requestTimeoutsRef.current[temperatureId];
      if (timeoutId !== undefined) {
        window.clearTimeout(timeoutId);
        requestTimeoutsRef.current[temperatureId] = undefined;
      }
    };

    temperatureResTopic.subscribe(cb);
    return () => temperatureResTopic?.unsubscribe(cb);
  }, [rerenderCount]);

  useEffect(() => {
    return () => {
      for (const timeoutId of Object.values(requestTimeoutsRef.current)) {
        if (timeoutId !== undefined) {
          window.clearTimeout(timeoutId);
        }
      }
    };
  }, []);

  const requestTemperature = (temperatureId: number) => {
    setPendingIds((current) => (current.includes(temperatureId) ? current : [...current, temperatureId]));
    const previousTimeoutId = requestTimeoutsRef.current[temperatureId];
    if (previousTimeoutId !== undefined) {
      window.clearTimeout(previousTimeoutId);
    }
    requestTimeoutsRef.current[temperatureId] = window.setTimeout(() => {
      setPendingIds((current) => current.filter((id) => id !== temperatureId));
      requestTimeoutsRef.current[temperatureId] = undefined;
    }, 3000);
    temperatureReqTopic?.publish({ data: temperatureId });
  };

  return (
    <>
      {readings.map((reading) => {
        const isPending = pendingIds.includes(reading.id);
        const statusClass =
          reading.status === 'error'
            ? styles['wexlab-status-error']
            : reading.status === 'ok'
              ? styles['wexlab-status-ok']
              : styles['wexlab-status-idle'];

        return (
          <div key={reading.id}>
            <div className={styles['wexlab-field-header']}>{reading.label}</div>
            <div className={styles['wexlab-row']}>
              <Label className={`${styles['wexlab-status-label']} ${statusClass}`}>
                <FontAwesomeIcon icon={faTemperatureThreeQuarters} />
              </Label>
              <Label color='var(--dark-background)' className={`${styles['wexlab-selectable']} ${styles['wexlab-temperature-value']}`}>
                {reading.value !== null ? `${reading.value.toFixed(2)} °C` : '---'}
              </Label>
              <Button tooltip={`Request temperature`} onClick={() => requestTemperature(reading.id)}>
                <FontAwesomeIcon className={isPending ? styles['wexlab-loading-icon'] : undefined} icon={faArrowRotateRight} />
              </Button>
            </div>
          </div>
        );
      })}
    </>
  );
}
