import styles from './ueuos.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import {
  SetUeuosColorRequest,
  SetUeuosStateRequest,
  SetUeuosEffectRequest,
  ColorRGBA,
  SET_UEUOS_STATE_OFF,
  SET_UEUOS_EFFECT_BOOT,
  SET_UEUOS_STATE_AUTONOMY,
  SET_UEUOS_STATE_TELEOP,
  SET_UEUOS_EFFECT_RAINBOW,
  SET_UEUOS_STATE_FINISHED
} from '../common/ros-interfaces';
import {
  faFillDrip,
  faMagic,
  faCube,
  faRainbow,
  faPowerOff,
  faHashtag,
  faRobot,
  faGamepad,
  faFlagCheckered
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useRef, useState } from 'react';
import { HexColorPicker } from 'react-colorful';
import { Service } from 'roslib';

import Button from '../components/button';
import Checkbox from '../components/checkbox';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';

const PANEL_TYPES = ['color', 'state', 'effect'] as const;
type PanelType = (typeof PANEL_TYPES)[number];

type Props = {
  props: {
    panelType: PanelType;
    color: string;
    state: number;
    effect: number;
  };
};

let setColorService: Service<SetUeuosColorRequest, {}> = null;
let setEffectService: Service<SetUeuosEffectRequest, {}> = null;
let setStateService: Service<SetUeuosStateRequest, {}> = null;
window.addEventListener('ros-connect', () => {
  setColorService = new Service<SetUeuosColorRequest, {}>({
    ros: ros,
    name: '/ueuos/set_color',
    serviceType: 'kalman_interfaces/SetColor'
  });

  setEffectService = new Service<SetUeuosEffectRequest, {}>({
    ros: ros,
    name: '/ueuos/set_effect',
    serviceType: 'kalman_interfaces/SetEffect'
  });

  setStateService = new Service<SetUeuosStateRequest, {}>({
    ros: ros,
    name: '/ueuos/set_state',
    serviceType: 'kalman_interfaces/SetState'
  });
});

function hexToColorRGB(hexColor: string): ColorRGBA {
  let raw = hexColor.trim().replace(/^#/, '').toUpperCase();

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
  let raw = hexColor.trim().replace(/^#/, '').toUpperCase();

  let r, g, b;
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

  // Compute luminance (brightness)
  let brightness = 0.299 * r + 0.587 * g + 0.114 * b;

  // If bright, return gray; otherwise, return white
  return brightness > 128 ? '#111111' : '#EEEEEE';
}

export default function Ueuos({ props }: Props) {
  if (props.panelType === undefined) {
    props.panelType = 'color';
  }
  if (props.color === undefined) {
    props.color = '#000080';
  }
  if (props.state === undefined) {
    props.state = SET_UEUOS_STATE_OFF;
  }
  if (props.effect === undefined) {
    props.effect = SET_UEUOS_EFFECT_BOOT;
  }

  const colorInputRef = useRef<Input>();

  const [panelType, setPanelType] = useState(props.panelType);

  const [color, setColor] = useState(props.color);
  const [effect, setEffect] = useState(props.effect);
  const [state, setState] = useState(props.state);

  const [colorInputValue, setColorInputValue] = useState(color.replace('#', ''));

  const [automaticallySynchronized, setAutomaticallySynchronized] = useState(false);

  const sendUeuosRequest = () => {
    if (!setColorService || !setStateService || !setEffectService) {
      alertsRef.current?.pushAlert(
        'Failed to update the UEUOS. Please make sure that ROS is connected and "/ueuos" services are available.',
        'error'
      );
      return;
    }

    if (panelType === 'color') {
      // Parse string from input to color object
      let colorObject: ColorRGBA;
      try {
        colorObject = hexToColorRGB(color);
      } catch (e) {
        alertsRef.current?.pushAlert(e.message ?? "Failed to set UEUOS's color:\n" + e, 'error');
        return;
      }

      // Send it to ROS
      const req: SetUeuosColorRequest = { color: colorObject };
      setColorService.callService(
        req,
        () => (props.color = color),
        (e) => alertsRef.current?.pushAlert("Failed to set UEUOS's color:\n" + e, 'error')
      );
    }

    if (panelType === 'effect') {
      const req: SetUeuosEffectRequest = { effect };
      setEffectService.callService(
        req,
        () => (props.effect = effect),
        (e) => alertsRef.current?.pushAlert("Failed to set UEUOS's effect:\n" + e, 'error')
      );
    }

    if (panelType === 'state') {
      const req: SetUeuosStateRequest = { state };
      setStateService.callService(
        req,
        () => (props.state = state),
        (e) => alertsRef.current?.pushAlert("Failed to set UEUOS's state:\n" + e, 'error')
      );
    }
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
      setColor('#' + value);

      if (automaticallySynchronized) {
        sendUeuosRequest();
      }
    }
  };

  const STATES: any = [
    {
      icon: faPowerOff,
      text: 'OFF',
      value: SET_UEUOS_STATE_OFF
    },
    {
      icon: faRobot,
      text: 'AUTONOMY',
      value: SET_UEUOS_STATE_AUTONOMY
    },
    {
      icon: faGamepad,
      text: 'TELEOP',
      value: SET_UEUOS_STATE_TELEOP
    },
    {
      icon: faFlagCheckered,
      text: 'FINISHED',
      value: SET_UEUOS_STATE_FINISHED
    }
  ];
  const EFFECTS: any = [
    {
      icon: faPowerOff,
      text: 'BOOT',
      value: SET_UEUOS_EFFECT_BOOT
    },
    {
      icon: faRainbow,
      text: 'RAINBOW',
      value: SET_UEUOS_EFFECT_RAINBOW
    }
  ];

  return (
    <div className={styles['ueuos']}>
      <div className={styles['ueuos-rows']}>
        <div className={styles['ueuos-row']}>
          <Dropdown
            className={styles['ueuos-row-item']}
            tooltip='Select UEUOS settings panel.'
            items={[
              {
                icon: faFillDrip,
                text: 'Color',
                value: 'color' as PanelType
              },
              {
                icon: faCube,
                text: 'State',
                value: 'state' as PanelType
              },
              {
                icon: faMagic,
                text: 'Effect',
                value: 'effect' as PanelType
              }
            ]}
            defaultItemIndex={PANEL_TYPES.indexOf(panelType)}
            onChange={(i, v) => {
              setPanelType(v);
              props.panelType = v;
            }}
          />
        </div>

        {panelType === 'color' && (
          <>
            <div className={styles['ueuos-row']}>
              <HexColorPicker
                className={`${styles['ueuos-row-item']} ${styles['ueuos-color-picker']}`}
                color={color}
                onChange={(value) => {
                  setColor(value);
                  colorInputRef.current?.setValue(value.replace('#', ''));

                  if (automaticallySynchronized) {
                    sendUeuosRequest();
                  }
                }}
              />
            </div>
            <div className={styles['ueuos-row']}>
              <Label
                className={styles['ueuos-hex-label']}
                style={{ backgroundColor: color, color: hexToGrayscaleReversed(color) }}
              >
                <FontAwesomeIcon icon={faHashtag} />
              </Label>
              <Input
                ref={colorInputRef}
                defaultValue={color.replace('#', '')}
                maxLength={6}
                onChange={handleHexColorInput}
                onSubmit={() => sendUeuosRequest()}
                onBlur={() => colorInputRef.current?.setValue(color.replace('#', ''))}
              />
            </div>
          </>
        )}

        {panelType === 'state' && (
          <div className={styles['ueuos-row']}>
            <Dropdown
              className={styles['ueuos-row-item']}
              tooltip='Select UEUOS state.'
              items={STATES}
              defaultItemIndex={STATES.findIndex((s) => s.value === state)}
              onChange={(i, value) => {
                setState(value);

                if (automaticallySynchronized) {
                  sendUeuosRequest();
                }
              }}
            />
          </div>
        )}

        {panelType === 'effect' && (
          <div className={styles['ueuos-row']}>
            <Dropdown
              className={styles['ueuos-row-item']}
              tooltip='Select UEUOS effect.'
              items={EFFECTS}
              defaultItemIndex={EFFECTS.findIndex((e) => e.value === effect)}
              onChange={(i, value) => {
                setEffect(value);

                if (automaticallySynchronized) {
                  sendUeuosRequest();
                }
              }}
            />
          </div>
        )}

        <div className={styles['ueuos-row']}>
          <Checkbox
            checked={automaticallySynchronized}
            onChange={(newChecked) => setAutomaticallySynchronized(newChecked)}
            label={'Auto-Update UEUOS'}
          />
        </div>

        <div className={styles['ueuos-row']}>
          <Button
            tooltip='Send data to change the UEUOS color'
            className={
              styles['ueuos-row-item'] + (automaticallySynchronized ? ` ${styles['ueuos-button-disabled']}` : '')
            }
            disabled={automaticallySynchronized}
            onClick={() => {
              sendUeuosRequest();
            }}
          >
            Send
          </Button>
        </div>
      </div>
    </div>
  );
}
