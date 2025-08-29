import styles from './arm_autonomy.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import {
  ArmMissionRequest, 
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
  faFlagCheckered,
  faBullseye
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

import GizmoPanel, { Gizmo } from '../components/gizmo-panel';

const DEFAULT_GIZMOS: Gizmo[] = [
  { x: 0.15, y: 0.12, color: '#78c7ff' }, // top-left blue
  { x: 0.50, y: 0.12, color: '#78c7ff' }, // top-mid blue
  { x: 0.85, y: 0.12, color: '#78c7ff' }, // top-right blue
  { x: 0.60, y: 0.80, color: '#ff9aa5' }, // bottom-right red 1
  { x: 0.85, y: 0.80, color: '#ff9aa5' }, // bottom-right red 2
];


const PANEL_TYPES = ['color', 'state', 'effect'] as const;
type PanelType = (typeof PANEL_TYPES)[number];

type Props = {
  props: {
    panelType: PanelType;
    color: string;
    state: number;
    effect: number;
    goal_id: string | null;
    target?: { x: number; y: number } | null;
  };
};

let armMissionService: Action<ArmMissionRequest, ArmMissionFeedback, {}> = null;

window.addEventListener('ros-connect', () => {
  armMissionService = new Action<ArmMissionRequest, ArmMissionFeedback, {}>({
    ros,
    name: '/arm/arm_mission',
    actionType: 'kalman_interfaces/action/ArmMission'
  });
});

export default function ArmAutonomy({ props }: Props) {
  const [goal_id, setGoal_id] = useState(props.goal_id);

  // const [selectedTarget, setSelectedTarget] = useState<{ x: number; y: number } | null>(props.target ?? null);
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null);

  const [automaticallySynchronized, setAutomaticallySynchronized] = useState(false);

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
        <br/>

        {/* Have a picker here, xy center shall be in the top-left corner */}
        <div className={styles['ueuos-row']}>
          <GizmoPanel
            width={360}
            height={260}
            gizmos={DEFAULT_GIZMOS /* or a JSON string */}
            selectedIndex={selectedIndex}
            onSelect={setSelectedIndex}
          />
        </div>

        <div className={styles['ueuos-row']}>
          <Button
            tooltip='Send data to change the UEUOS color'
            className={
              styles['ueuos-row-item']
            }
            disabled={selectedIndex == null}
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
