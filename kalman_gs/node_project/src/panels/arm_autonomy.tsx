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
  faBullseye,
  faArrowsUpDown,
  faArrowsLeftRight,
  faArrowDown,
  faArrowRight
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useRef, useState } from 'react';
import { HexColorPicker } from 'react-colorful';
import { Action, Service, Topic } from 'roslib';

import Button from '../components/button';
import Checkbox from '../components/checkbox';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';

import GizmoPanel, { Gizmo } from '../components/gizmo-panel';

const INITIAL_GIZMOS: Gizmo[] = [
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

// TODO: move this type somewhere else
export type GeometryMsgsPoint = {
  x?: number;
  y?: number;
  z?: number;
};

let armMissionService: Action<ArmMissionRequest, ArmMissionFeedback, {}> = null;
let armPointTopic: Topic<GeometryMsgsPoint> = null

window.addEventListener('ros-connect', () => {
  armMissionService = new Action<ArmMissionRequest, ArmMissionFeedback, {}>({
    ros,
    name: '/arm/arm_mission',
    actionType: 'kalman_interfaces/action/ArmMission'
  });
  armPointTopic = new Topic<GeometryMsgsPoint>({
    ros,
    name: '/arm/uv_point',
    messageType: 'geometry_msgs/Point',
  })
});

export default function ArmAutonomy({ props }: Props) {
  const [goal_id, setGoal_id] = useState(props.goal_id);

  const [gizmos, setGizmos] = useState<Gizmo[]>(INITIAL_GIZMOS);
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null);

  const posVRef = useRef<Input>(); // v = vertical = y
  const posURef = useRef<Input>(); // u = horizontal = x

  const handleGizmoSelect = (i: number) => {
    setSelectedIndex(i);
    const g = gizmos[i];
    posURef.current?.setValue(g.x);
    posVRef.current?.setValue(g.y);
  };

  const handleInputsToLayout = () => {
    if (selectedIndex == null) return;
    const uStr = posURef.current?.getValue?.();
    const vStr = posVRef.current?.getValue?.();
    const u = parseFloat(String(uStr));
    const v = parseFloat(String(vStr));
    if (!Number.isFinite(u) || !Number.isFinite(v)) return;

    setGizmos((prev) =>
      prev.map((g, i) =>
        i === selectedIndex ? { ...g, x: u, y: v } : g
      )
    );
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

  const sendArmPointRequest = () => {
    if (!armPointTopic) {
      alertsRef.current?.pushAlert(
        'Failed to send arm point. Ensure ROS is connected and /arm/uv_point is available.',
        'error'
      );
      return;
    }
    if (selectedIndex == null) {
      alertsRef.current?.pushAlert('Select a gizmo first.', 'warning');
      return;
    }
    const g = gizmos[selectedIndex];
    const msg: GeometryMsgsPoint = { x: g.x, y: g.y, z: 0.2 };
    armPointTopic.publish(msg);
    alertsRef.current?.pushAlert(`Sent point: u=${g.x.toFixed(3)}, v=${g.y.toFixed(3)}`, 'success');
  };


  return (
    <div className={styles['ueuos']}>
      <div className={styles['ueuos-rows']}>
        {goal_id == null ? (
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
        ) : (
          // TODO : I would prefer if at mission trigger, the LocatePanel button split into two, one for cancel, second for re-trigger
          <div className={styles['ueuos-row']}>
            <Button
              tooltip='Cancel panel location mission'
              className={
                styles['ueuos-row-item']
              }
              onClick={() => {
                cancelArmMissionRequest();
              }}
            >
              Cancel locate panel
            </Button>
          </div>
        )}

        <br/>

        <div className={styles['ueuos-row']}>
          <GizmoPanel
            width={360}
            height={260}
            gizmos={gizmos}
            selectedIndex={selectedIndex}
            onSelect={handleGizmoSelect}
          />
        </div>

      {/* Inputs control layout */}
        <div className={styles['ueuos-row']}>
          <Label className={styles['latlon-label']}><FontAwesomeIcon icon={faArrowDown} /></Label>
          <Input
            type="float"
            placeholder="Vertical (v)"
            ref={posVRef}
            disabled={selectedIndex == null}
            step={0.01 as any}
            onChange={handleInputsToLayout}
          />
          <Label className={styles['latlon-label']}><FontAwesomeIcon icon={faArrowRight} /></Label>
          <Input
            type="float"
            placeholder="Horizontal (u)"
            ref={posURef}
            disabled={selectedIndex == null}
            step={0.01 as any}
            onChange={handleInputsToLayout}
          />
        </div>

        <div className={styles['ueuos-row']}>
          <Button
            className={styles['ueuos-row-item']}
            disabled={selectedIndex == null}
            onClick={sendArmPointRequest}
          >
            Send
          </Button>
        </div>
      </div>
    </div>
  );
}
