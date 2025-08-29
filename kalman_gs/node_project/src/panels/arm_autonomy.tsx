import styles from './arm_autonomy.module.css';

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
  SET_UEUOS_STATE_FINISHED,
  ArmMissionRequest, // ArmMission,
  ArmMissionFeedback
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
import { Action, Service } from 'roslib';

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
    goal_id: string | null;
  };
};

let armMissionService: Action<ArmMissionRequest, ArmMissionFeedback, {}> = null;

let setColorService: Service<SetUeuosColorRequest, {}> = null;
window.addEventListener('ros-connect', () => {
  // FIXME: remove when no longer useful as a reference
  setColorService = new Service<SetUeuosColorRequest, {}>({
    ros: ros,
    name: '/ueuos/set_color',
    serviceType: 'kalman_interfaces/SetColor'
  });

  armMissionService = new Action<ArmMissionRequest, ArmMissionFeedback, {}>({
    ros,
    name: '/arm/arm_mission',
    actionType: 'kalman_interfaces/action/ArmMission'
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

export default function ArmAutonomy({ props }: Props) {
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
  const [goal_id, setGoal_id] = useState(props.goal_id);

  const [colorInputValue, setColorInputValue] = useState(color.replace('#', ''));

  const [automaticallySynchronized, setAutomaticallySynchronized] = useState(false);

  const sendUeuosRequest = () => {
    if (!setColorService) {
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
  };

  const sendArmMissionRequest = () => {
    // FIXME: re-starting mission after it's finished once throws this error
    if (!armMissionService) {
      alertsRef.current?.pushAlert(
        'Failed to send arm mission request. Please make sure that ROS is connected and /arm/arm_mission services are available.',
        'error'
      );
      return;
    }
    const req: ArmMissionRequest = { command_id: 1 };
    let goal: string = armMissionService.sendGoal(
      req,
      // () => (setGoal_id(null)),
      () => ({}), // TODO: add proper handling of the result
      (feedback) => (alertsRef.current?.pushAlert('Feedback: ' + feedback.progress, 'warning')),
      (error) => (alertsRef.current?.pushAlert('Error: ' + error, 'warning')),
    );
    setGoal_id(goal);
  };

  const cancelArmMissionRequest = () => {
    if (!armMissionService) {
      alertsRef.current?.pushAlert(
        'Failed to send arm mission request. Please make sure that ROS is connected and /arm/arm_mission services are available.',
        'error'
      );
      return;
    }
    if (goal_id == null) {
      alertsRef.current?.pushAlert(
        'Failed to cancel arm mission goal. Goal handle is null',
        'error'
      );
      return;

    }
    armMissionService.cancelGoal(goal_id);
    setGoal_id(null);
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
        {goal_id == null && (
          <div className={styles['ueuos-row']}>
            <Button
              tooltip='Trigger panel location mission'
              className={
                styles['ueuos-row-item']
              }
              // disabled={automaticallySynchronized}
              onClick={() => {
                sendArmMissionRequest();
              }}
            >
              Locate panel
            </Button>
          </div>
        )}

        {goal_id != null && (
          // TODO : I would prefer if at mission trigger, the LocatePanel button split into two, one for cancel, second for re-trigger
          <div className={styles['ueuos-row']}>
            <Button
              tooltip='Cancel panel location mission'
              className={
                styles['ueuos-row-item']
              }
              // disabled={automaticallySynchronized}
              onClick={() => {
                cancelArmMissionRequest();
              }}
            >
              Cancel locate panel
            </Button>
          </div>
        ) }
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
              sendArmMissionRequest();
            }}
          >
            Send
          </Button>
        </div>
      </div>
    </div>
  );
}
