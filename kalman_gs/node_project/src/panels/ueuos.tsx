import styles from './ueuos.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { SetColorRequest, SetStateRequest, SetEffectRequest, ColorRGB } from '../common/ros-interfaces';
import { faFillDrip, faMagic, faCube, faRainbow, faPowerOff, faHashtag } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useRef, useState } from 'react';
import { HexColorPicker } from 'react-colorful';
import { Service } from 'roslib';

import Button from '../components/button';
import Checkbox from '../components/checkbox';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';

enum PanelType {
  COLOR = 0,
  STATE = 1,
  EFFECT = 2
}

enum UeuosState {
  OFF = 0,
  AUTONOMY = 1,
  TELEOP = 2,
  FINISHED = 3
}

enum UeuosEffect {
  BOOT = 0,
  RAINBOW = 1
}

type Props = {
  props: {
    panelType: number;
  };
};

let setColorService: Service<SetColorRequest, {}> = null;
let setEffectService: Service<SetEffectRequest, {}> = null;
let setStateService: Service<SetStateRequest, {}> = null;
window.addEventListener('ros-connect', () => {
  setColorService = new Service<SetColorRequest, {}>({
    ros: ros,
    name: '/ueuos/set_color',
    serviceType: 'kalman_interfaces/SetColor'
  });

  setEffectService = new Service<SetEffectRequest, {}>({
    ros: ros,
    name: '/ueuos/set_effect',
    serviceType: 'kalman_interfaces/SetEffect'
  });

  setStateService = new Service<SetStateRequest, {}>({
    ros: ros,
    name: '/ueuos/set_state',
    serviceType: 'kalman_interfaces/SetState'
  });
});

function hexToColorRGB(hexColor: string): ColorRGB {
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
    props.panelType = parseInt(localStorage.getItem('ueuos-panel-type') ?? '0');
  }

  const colorInputRef = useRef<Input>();

  const [panelType, setPanelType] = useState(props.panelType);

  const [color, setColor] = useState(localStorage.getItem('ueuos-color') ?? '000080');
  const [effect, setEffect] = useState(UeuosEffect[localStorage.getItem('ueuos-effect')] ?? UeuosEffect.BOOT);
  const [state, setState] = useState(UeuosEffect[localStorage.getItem('ueuos-state')] ?? UeuosState.OFF);

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

    if (panelType === PanelType.COLOR) {
      // Parse string from input to color object
      let colorObject: ColorRGB;
      try {
        colorObject = hexToColorRGB(color);
      } catch (e) {
        alertsRef.current?.pushAlert(e.message ?? "Failed to set UEUOS's color.", 'error');
        return;
      }

      // Send it to ROS
      const req: SetColorRequest = { color: colorObject };
      setColorService.callService(
        req,
        () => localStorage.setItem('ueuos-color', color),
        () => alertsRef.current?.pushAlert("Failed to set UEUOS's color.", 'error')
      );
    }

    if (panelType === PanelType.EFFECT) {
      const req: SetEffectRequest = { effect };
      setEffectService.callService(
        req,
        () => localStorage.setItem('ueuos-effect', effect.toString()),
        () => alertsRef.current?.pushAlert("Failed to set UEUOS's effect.", 'error')
      );
    }

    if (panelType === PanelType.STATE) {
      const req: SetStateRequest = { state };
      setStateService.callService(
        req,
        () => localStorage.setItem('ueuos-state', state.toString()),
        () => alertsRef.current?.pushAlert("Failed to set UEUOS's state.", 'error')
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
                value: PanelType.COLOR
              },
              {
                icon: faMagic,
                text: 'Effect',
                value: PanelType.EFFECT
              },
              {
                icon: faCube,
                text: 'State',
                value: PanelType.STATE
              }
            ]}
            defaultItemIndex={panelType}
            onChange={(i) => {
              setPanelType(i);
              props.panelType = i;

              localStorage.setItem('ueuos-panel-type', `${i}`);
            }}
          />
        </div>

        {panelType === PanelType.COLOR && (
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
              <Label style={{ backgroundColor: color, color: hexToGrayscaleReversed(color) }}>
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

        {panelType === PanelType.EFFECT && (
          <div className={styles['ueuos-row']}>
            <Dropdown
              className={styles['ueuos-row-item']}
              tooltip='Select UEUOS effect.'
              items={[
                {
                  icon: faPowerOff,
                  text: 'BOOT',
                  value: UeuosEffect.BOOT
                },
                {
                  icon: faRainbow,
                  text: 'RAINBOW',
                  value: UeuosEffect.RAINBOW
                }
              ]}
              onChange={(i, value) => {
                setEffect(value);

                if (automaticallySynchronized) {
                  sendUeuosRequest();
                }
              }}
            />
          </div>
        )}

        {panelType === PanelType.STATE && (
          <div className={styles['ueuos-row']}>
            <Dropdown
              className={styles['ueuos-row-item']}
              tooltip='Select UEUOS state.'
              items={[
                {
                  icon: faPowerOff,
                  text: 'OFF',
                  value: UeuosState.OFF
                },
                {
                  icon: faPowerOff,
                  text: 'AUTONOMY',
                  value: UeuosState.AUTONOMY
                },
                {
                  icon: faPowerOff,
                  text: 'TELEOP',
                  value: UeuosState.TELEOP
                },
                {
                  icon: faPowerOff,
                  text: 'FINISHED',
                  value: UeuosState.FINISHED
                }
              ]}
              onChange={(i, value) => {
                setState(value);

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
