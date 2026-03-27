import styles from './wexlab.module.css';

import { ros } from '../common/ros';
import { ColorRGBA, WExLabHeaterCfg, WExLabLedAll, WExLabLedSingle } from '../common/ros-interfaces';
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
  faHashtag
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
    ledTarget?: 'all' | 0 | 1 | 2;
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
};

type HeaterCfgKey = 'heaterThermostatMin' | 'heaterThermostatMax' | 'heaterPowerMain' | 'heaterPowerLid';

type HeaterField = {
  key: HeaterCfgKey;
  placeholder: string;
};

const HEATER_FIELDS: HeaterField[] = [
  { key: 'heaterThermostatMin', placeholder: 'Thermostat Min' },
  { key: 'heaterThermostatMax', placeholder: 'Thermostat Max' },
  { key: 'heaterPowerMain', placeholder: 'Power Main' },
  { key: 'heaterPowerLid', placeholder: 'Power Lid' }
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

function ValueEditor({ min, max, step = 1, placeholder, value = 0, onValueChange, onSend }: ValueEditorProps) {
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
    const normalized = normalizeNumber(value, -100, 100, 0);
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
    </>
  );
}

function NumberInputRow({
  inputRef,
  inputValue,
  className,
  placeholder,
  onChange,
  onSubmit,
  onBlur,
  onDecrease,
  onIncrease
}: {
  inputRef: RefObject<Input>;
  inputValue: number;
  className?: string;
  placeholder?: string;
  onChange: (text: string) => void;
  onSubmit: (text: string) => void;
  onBlur: () => void;
  onDecrease: () => void;
  onIncrease: () => void;
}) {
  return (
    <div className={styles['wexlab-row']}>
      <Button className={styles['wexlab-step-button']} tooltip='Decrease value by 1' onClick={onDecrease}>
        <FontAwesomeIcon icon={faMinus} />
      </Button>
      <Input
        ref={inputRef}
        type='float'
        className={className}
        placeholder={placeholder}
        defaultValue={String(inputValue)}
        onChange={onChange}
        onSubmit={onSubmit}
        onBlur={onBlur}
      />
      <Button className={styles['wexlab-step-button']} tooltip='Increase value by 1' onClick={onIncrease}>
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
        <NumberInputRow
          key={field.key}
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
      ))}

      <div className={styles['wexlab-row']}>
        <Button className={styles['wexlab-row-item']} tooltip='Send heater configuration' onClick={publishCfg}>
          <FontAwesomeIcon icon={faPaperPlane} />
          &nbsp;&nbsp;Send
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
  target: 'all' | 0 | 1 | 2;
  color: string;
  onTargetChange: (target: 'all' | 0 | 1 | 2) => void;
  onColorChange: (color: string) => void;
}) {
  const colorInputRef = useRef<Input>(null);
  const [colorInputValue, setColorInputValue] = useState(color.replace('#', ''));

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
      led_id: target,
      color: colorObject
    });
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

  const ledOptions: Array<'all' | 0 | 1 | 2> = ['all', 0, 1, 2];

  return (
    <>
      <div className={styles['wexlab-row']}>
        {ledOptions.map((option) => (
          <Button
            key={option}
            className={styles['wexlab-row-item']}
            tooltip={`Select LED target ${option}`}
            disabled={target === option}
            onClick={() => onTargetChange(option)}
          >
            {option === 'all' ? 'All' : option}
          </Button>
        ))}
      </div>

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
    };

    weightResTopic.subscribe(cb);
    return () => weightResTopic?.unsubscribe(cb);
  }, [rerenderCount]);

  const requestWeight = () => {
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

  const tareOffset = tareHistory.reduce((sum, value) => sum + value, 0);
  const displayWeight = weight !== null ? weight - tareOffset : null;

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
          {displayWeight !== null ? `${displayWeight.toFixed(2)} g` : '--- g'}
        </Label>
        <Button tooltip='Request weight measurement' onClick={requestWeight}>
          <FontAwesomeIcon icon={faArrowRotateRight} />
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
