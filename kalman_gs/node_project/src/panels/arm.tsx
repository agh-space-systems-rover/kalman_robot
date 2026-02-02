import styles from './arm.module.css';

import {
  setLinearScaleTo,
  setRotationalScaleTo,
  lastServoLinearScale,
  lastServoRotationalScale,
  abortPose,
  sendPoseRequest,
  keepAlivePose,
  lastStatusPose,
  keepAliveTrajectory,
  sendTrajectoryRequest,
  abortTrajectory,
  lastStatusTrajectory,
  toggleArmAxisLock,
  armAxesLocks
} from '../common/arm';
import { armJointsLocks, currentAxisLockFocus, toggleArmJointLock } from '../common/gamepad-arming';
import predefinedPoses from '../common/predefined-arm-poses';
import '../common/predefined-arm-trajectories';
import predefinedArmTrajectories from '../common/predefined-arm-trajectories';
import { ros } from '../common/ros';
import { JointState } from '../common/ros-interfaces';
import { faDownload, faLock, faLockOpen, faSave, faTrash, faUpload } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useState, useEffect, useCallback } from 'react';
import { Topic } from 'roslib';
import Button from '../components/button';

let lastJointState: JointState | null = null;

interface ArmPose{
  id: number;
  name: string;
  path: string;
  joints: number[];
  joints_set: number[];
  joints_checked: number[];
  joints_reversed?: number[];
  safe_previous_poses: number[];
};

window.addEventListener('ros-connect', () => {
  const jointTopic = new Topic({
    ros: ros,
    name: '/arm_controllers/joint_states',
    messageType: 'sensor_msgs/JointState'
  });

  jointTopic.subscribe((msg: JointState) => {
    lastJointState = msg;
    window.dispatchEvent(new Event('joint-state'));
  });
});

const jointLimitsRad = [
  { min: -6.2815926, max: 6.2815926 },
  { min: -3.1415926, max: 3.1415926 },
  { min: -2.88, max: 2.88 },
  { min: -6.4, max: 6.4 },
  { min: -1.75, max: 1.75 },
  { min: -3.4032, max: 3.4032 }
];

function rad2deg(rad: number) {
  return (rad * 180) / Math.PI;
}

function getJointNames() {
  return Array.from({ length: 6 }, (_, i) => (
    <div className={styles['joint-name']} key={i}>
      Joint {i + 1}:
    </div>
  ));
}

function getNamesAndValues() {
  let namesAndValues = lastJointState
    ? lastJointState.name.map((name, i) => {
        return { name: name, value: lastJointState!.position[i] };
      })
    : [];

  namesAndValues.sort((a, b) => {
    return a.name.localeCompare(b.name);
  });

  return namesAndValues.slice(0, 6);
}

function isCloseEnough(jointsA: number[], jointsB: number[], maxDistance: number, checkedJoints: number[] = []) {
  if (jointsA.length !== jointsB.length) {
    return false;
  }
  for (let i = 0; i < jointsA.length; i++) {
    if (i + 1 in checkedJoints && Math.abs(jointsA[i] - jointsB[i]) > maxDistance) {
      return false;
    }
  }
  return true;
}

function ArmStatus({ // MARK -- ARM STATUS
    editMode
  }: { 
    editMode: boolean 
  }) {
  const [rerenderCount, setRerenderCount] = useState(0);
  const [linearScale, setLinearScale] = useState<number | null>(lastServoLinearScale);
  const [rotationalScale, setRotationalScale] = useState<number | null>(lastServoRotationalScale);

  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
    setLinearScale(lastServoLinearScale);
    setRotationalScale(lastServoRotationalScale);
  }, []);

  useEffect(() => {
    window.addEventListener('joint-state', rerender);
    window.addEventListener('servo-linear-scale', rerender);
    window.addEventListener('servo-rotational-scale', rerender);
    window.addEventListener('arm-axis-lock-update', rerender);
    window.addEventListener('arm-joint-lock-update', rerender);
    return () => {
      window.removeEventListener('joint-state', rerender);
      window.removeEventListener('servo-linear-scale', rerender);
      window.removeEventListener('servo-rotational-scale', rerender);
      window.removeEventListener('arm-axis-lock-update', rerender);
      window.removeEventListener('arm-joint-lock-update', rerender);
    };
  }, []);

  const handleLinearScaleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    if (!isNaN(value) && value >= 0 && value <= 1) {
      setLinearScale(value);
      setLinearScaleTo(value); // Update the external value
    }
  };

  const handleRotationalScaleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    if (!isNaN(value) && value >= 0 && value <= 1) {
      setRotationalScale(value);
      setRotationalScaleTo(value); // Update the external value
    }
  };

  const namesAndValues = getNamesAndValues();
  const jointNames = getJointNames();

  const jointRangeLeft = jointLimitsRad.map((limit, i) => (
    <div className={styles['joint-range']} key={i}>
      {rad2deg(limit.min).toFixed(0)}..
    </div>
  ));

  const WARN_THRESHOLD = (20 * Math.PI) / 180;
  const ERROR_THRESHOLD = (5 * Math.PI) / 180;

  const distancesFromLimits = namesAndValues.map((joint, i) => {
    if (i > 5) return 0;
    return Math.min(joint.value - jointLimitsRad[i].min, jointLimitsRad[i].max - joint.value);
  });

  const stylesFromLimits = distancesFromLimits.map((distance) => {
    if (distance < ERROR_THRESHOLD) {
      return styles['error'] + ' ';
    }
    if (distance < WARN_THRESHOLD) {
      return styles['warn'] + ' ';
    }
    return '';
  });

  const jointValues = Array.from({ length: 6 }, (_, i) => (
    <div className={(stylesFromLimits[i] ? stylesFromLimits[i] : '') + styles['joint-value']} key={i}>
      {lastJointState ? rad2deg(namesAndValues[i].value).toFixed(0) : 'N/A'}
    </div>
  ));

  const jointRangeRight = jointLimitsRad.map((limit, i) => (
    <div className={styles['joint-range']} key={i}>
      ..{rad2deg(limit.max).toFixed(0)}
    </div>
  ));

  const jointLocks = Array.from({ length: 6 }, (_, i) => (
    <div
      className={`${styles['joint-lock']} ${currentAxisLockFocus == i + 1 ? styles['lock-selected'] : ''}`}
      key={i + armJointsLocks[`joint_${i + 1}`] * 10}
      onClick={() => {
        toggleArmJointLock(`joint_${i + 1}`);
      }}
    >
      {armJointsLocks[`joint_${i + 1}`] ? <FontAwesomeIcon icon={faLock} /> : <FontAwesomeIcon icon={faLockOpen} />}
    </div>
  ));

  const getAxisLockIcon = (axis: string) => (
    <div className={`${styles['joint-lock']}`} key={axis + armAxesLocks[axis]} onClick={() => toggleArmAxisLock(axis)}>
      {armAxesLocks[axis] ? <FontAwesomeIcon icon={faLock} /> : <FontAwesomeIcon icon={faLockOpen} />}
    </div>
  );

  const saveCurrentPose = () => {
    const poseName = prompt("Give the name of new pose:", `Poza ${new Date().toLocaleTimeString()}`);
    if (!poseName) return;

    const currentValues = namesAndValues.map(joint => joint.value);
    
    const savedPosesRaw = localStorage.getItem('custom_arm_poses');
    const savedPoses = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];
    
    
    const newPose: ArmPose= {
      id: Date.now(), // FIXME unique ID
      name: poseName,
      path: "TODO", // TODO generating yaml with pose data
      joints: currentValues,
      joints_set: [1, 2, 3, 4, 5, 6],
      joints_checked: [1, 2, 3, 4, 5, 6],
      safe_previous_poses: []
    };

    localStorage.setItem('custom_arm_poses', JSON.stringify([...savedPoses, newPose]));

    window.dispatchEvent(new Event('local-poses-update'));
    alert("Pose saved!");
  };

  return (
    <div className={styles['arm-status']}>
      <div className={styles['header-container']}>
        <h1 className={styles['status-header']}>Arm Status</h1>
        {editMode && (
        <button className={styles['save-pose-button']} onClick={saveCurrentPose}>
          <FontAwesomeIcon icon={faSave}/>
        </button>)}
      </div>
      <div className={styles['status']}>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>{jointLocks}</div>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>{jointNames}</div>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>{jointRangeLeft}</div>
        <div className={styles['joint-column']}>{jointValues}</div>
        <div className={styles['joint-column'] + ' ' + styles['align-right']}>{jointRangeRight}</div>
      </div>
      <h3 className={styles['scales-header']}>Scales</h3>
      <div className={styles['scales']}>
        <div className={styles['scale-column'] + ' ' + styles['align-left']}>
          <div className={styles['scale-name']}>Linear scale:</div>
          <div className={styles['scale-name']}>Rotational scale:</div>
        </div>
        <div className={styles['scale-column']}>
          <div className={styles['scale-value-holder']} key={lastServoLinearScale}>
            <input
              type='range'
              min='0'
              max='1'
              step='0.01'
              value={linearScale || 0}
              onChange={handleLinearScaleChange}
            />
            <div className={styles['scale-value'] + ' ' + styles['blink']}>{linearScale?.toFixed(2)}</div>
          </div>

          <div className={styles['scale-value-holder']} key={lastServoRotationalScale + 10}>
            <input
              type='range'
              min='0'
              max='1'
              step='0.01'
              value={rotationalScale || 0}
              onChange={handleRotationalScaleChange}
            />
            <div className={styles['scale-value'] + ' ' + styles['blink']}>{rotationalScale?.toFixed(2)}</div>
          </div>
        </div>
      </div>

      <h3 className={styles['scales-header']}>Axis locks</h3>
      <div className={styles['locks']}>
        <div className={styles['scale-column']}>
          <div className={styles['lock-name']}>X: </div>
          <div className={styles['lock-name']}>Y: </div>
          <div className={styles['lock-name']}>Z: </div>
        </div>
        <div className={styles['scale-column']}>
          {getAxisLockIcon('x')}
          {getAxisLockIcon('y')}
          {getAxisLockIcon('z')}
        </div>
        <div className={styles['scale-column']}>
          <div className={styles['lock-name']}>Roll: </div>
          <div className={styles['lock-name']}>Pitch: </div>
          <div className={styles['lock-name']}>Yaw: </div>
        </div>
        <div className={styles['scale-column']}>
          {getAxisLockIcon('roll')}
          {getAxisLockIcon('pitch')}
          {getAxisLockIcon('yaw')}
        </div>
      </div>
    </div>
  );
}

function poseJoints(jointValues) {
  return (
    <div className={styles['status']}>
      <div className={styles['joint-column'] + ' ' + styles['align-left']}>{getJointNames()}</div>
      <div className={styles['joint-column']}>{jointValues}</div>
    </div>
  );
}

function PoseRequester({ // MARK -- POSE REQUESTER
    editMode,
    onSelectPose
  }: {
    editMode: boolean;
    onSelectPose: (pose: ArmPose) => void;
  }) {
  const [rerenderCount, setRerenderCount] = useState(0);
  const [keepAlive, setKeepAlive] = useState(false);
  const [customPoses, setCustomPoses] = useState<ArmPose[]>([]); // własne pozy


  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, [setRerenderCount]);

  useEffect(() => {
    const loadCustomPoses = () => {
      const savedPosesRaw = localStorage.getItem('custom_arm_poses');
      const savedPoses: ArmPose[] = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];
      setCustomPoses(savedPoses);
    };

    loadCustomPoses();
    window.addEventListener('local-poses-update', loadCustomPoses);

    return () => {
      window.removeEventListener('local-poses-update', loadCustomPoses);
    };
  }, []);

  useEffect(() => {
    window.addEventListener('joint-state', rerender);
    window.addEventListener('pose-status', rerender);
    return () => {
      window.removeEventListener('joint-state', rerender);
      window.removeEventListener('pose-status', rerender);
    };
  }, [rerender]);

  useEffect(() => {
    const intervalId = setInterval(() => {
      if (keepAlive) {
        keepAlivePose();
      }
      if (lastStatusPose === 'ABORT_RECEIVED') {
        setKeepAlive(false);
      }
    }, 200);

    // Cleanup interval on component unmount
    return () => clearInterval(intervalId);
  }, []);

  const isStartingFromSafePose = (safePreviousPoses: number[]) => {
    for (let i = 0; i < safePreviousPoses.length; i++) {
      if (
        isCloseEnough(
          predefinedPoses.poses[safePreviousPoses[i]].joints,
          namesAndValues.map((joint) => joint.value),
          predefinedPoses.max_distance_rad,
          predefinedPoses.poses[safePreviousPoses[i]].joints_checked
        )
      ) {
        return true;
      }
    }
    return false;
  };

  const allPoses: ArmPose[] = [
    ...predefinedPoses.poses.map((p) => ({ 
      ...p, 
      isCustom: false 
    })),
    ...customPoses.map((p) => ({ 
      ...p, 
      isCustom: true 
    }))
  ];

  const [currentPoseId, setCurrentPoseId] = useState(0);
  const namesAndValues = getNamesAndValues();

  const currentPose = allPoses.find((p) => p.id === currentPoseId) || allPoses[0];

  const predefinedJointValues = currentPose.joints ?? [0, 0, 0, 0, 0, 0];

  const posesToSelect = allPoses.map((pose) => (
    <div
      key={pose.id}
      className={`${styles['pose-button']} ${styles['pose-option']} ${pose.id === currentPoseId ? styles['pose-selected'] : ''}`}
      onClick={() => {
        setCurrentPoseId(pose.id);
        if (editMode && pose.joints) onSelectPose(pose);
      }}
    >
      <div className={styles['pose-name']}>{pose.name}</div>
      <div
        className={`${styles['pose-indicator']} ${
          isCloseEnough(
            pose.joints ?? [],
            namesAndValues.map((j) => j.value),
            predefinedPoses.max_distance_rad,
            pose.joints_checked
          )
            ? styles['pose-ready']
            : styles['pose-not-ready']
        }`}
      />
    </div>
  ));

  let isJointSet: boolean[] = Array(6).fill(false);
  let isJointClose: boolean[] = Array(6).fill(false);
  let isJointChecked: boolean[] = Array(6).fill(false);

  if (currentPose) {
    currentPose.joints_set.forEach((value) => {
        isJointSet[value - 1] = true;
    });

    isJointClose = isJointClose.map((_, i) =>
        namesAndValues.length && predefinedJointValues
            ? Math.abs(predefinedJointValues[i] - namesAndValues[i].value) <= predefinedPoses.max_distance_rad
            : false
    );

    currentPose.joints_checked.forEach((value) => {
        isJointChecked[value - 1] = true;
    });
  }

  const PoseJoints = poseJoints(
    Array.from({ length: 6 }, (_, i) => (
      <div
        className={`${styles['joint-value']} ${!isJointSet[i] ? styles['joint-not-set'] : !isJointClose[i] && isJointChecked[i] ? styles['warn'] : ''}`}
        key={i}
      >
        {rad2deg(predefinedJointValues[i]).toFixed(0)}
      </div>
    ))
  );

  const closeEnough = currentPose && namesAndValues.length > 0 ? (
      isCloseEnough(
          predefinedJointValues,
          namesAndValues.map((joint) => joint.value),
          predefinedPoses.max_distance_rad,
          currentPose.joints_checked
      ) || isStartingFromSafePose(currentPose.safe_previous_poses)
  ) : false;

  const handleImportSinglePose = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const importedData = JSON.parse(event.target?.result as string);
        
        const importedArray = Array.isArray(importedData) ? importedData : [importedData];
        
        const savedPosesRaw = localStorage.getItem('custom_arm_poses');
        let existingPoses: ArmPose[] = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];

        importedArray.forEach((newPose) => {
          const poseToAdd: ArmPose = { ...newPose, isCustom: true };

          while (existingPoses.some(p => p.id === poseToAdd.id)) {
            poseToAdd.id = Date.now();
          }

          if (existingPoses.some(p => p.name === poseToAdd.name)) {
            poseToAdd.name = `${poseToAdd.name} (Imported)`;
          }

          existingPoses.push(poseToAdd);
        });

        localStorage.setItem('custom_arm_poses', JSON.stringify(existingPoses));
        
        window.dispatchEvent(new Event('local-poses-update'));
        alert(`Successfully imported ${importedArray.length} pose(s).`);
      } catch (err) {
        alert("Error: Invalid JSON format.");
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  };

  return (
    <div className={styles['pose-requester']}>
      <div className={styles['header-container']}>
        <h2 className={styles['pose-header']}>Pose Requester</h2>
        
        {editMode && (<label className={styles['import-label']}>
          <input 
            type="file" 
            accept=".json" 
            style={{ display: 'none' }} 
            onChange={handleImportSinglePose} 
          />
          <div className={styles['export-poses-button']} title="Import Pose JSON">
            <FontAwesomeIcon icon={faDownload} />
          </div>
        </label>)}
      </div>

      <div className={styles['pose-panel']}>
        {PoseJoints}
        <div className={styles['pose-options']}>{posesToSelect}</div>
      </div>
      <div className={styles['pose-buttons']}>
        <div
          className={`${styles['pose-button']} ${styles['pose-send']} ${
            closeEnough ? styles['send-ready'] : styles['send-not-ready']
          }`}
          onClick={() => {
            if (closeEnough) {
              setKeepAlive(true);
              keepAlivePose();
              sendPoseRequest(currentPoseId);
            }
          }}
        >
          {closeEnough ? 'Send Pose' : 'Cannot Send'} 
          {/* MARK #test sending pose */}
        </div>
        <div
          className={`${styles['pose-button']} ${styles['pose-abort']}`}
          onClick={() => {
            abortPose();
          }}
        >
          Abort
        </div>
      </div>
      <div className={styles['pose-status']}>Status: {lastStatusPose}</div>
    </div>
  );
}

function TrajectoryRequester() { // MARK -- TRAJECTORY REQUESTER
  const [rerenderCount, setRerenderCount] = useState(0);
  const [keepAlive, setKeepAlive] = useState(false);
  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, [setRerenderCount]);

  useEffect(() => {
    window.addEventListener('joint-state', rerender);
    window.addEventListener('trajectory-status', rerender);
    return () => {
      window.removeEventListener('joint-state', rerender);
      window.removeEventListener('trajectory-status', rerender);
    };
  }, [rerender]);

  useEffect(() => {
    const intervalId = setInterval(() => {
      if (keepAlive) {
        keepAliveTrajectory();
      }
      if (lastStatusTrajectory === 'ABORT_RECEIVED') {
        setKeepAlive(false);
      }
    }, 200);

    // Cleanup interval on component unmount
    return () => clearInterval(intervalId);
  }, []);

  const [currentTrajectoryId, setCurrentTrajectoryId] = useState(0);

  const namesAndValues = getNamesAndValues();

  const predefinedJointValues =
    predefinedArmTrajectories.START_JOINTS[
      predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[currentTrajectoryId].name
    ];

  const trajectoriesToSelect = predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories.map((trajectory, i) => (
    <div
      key={trajectory.id}
      className={`${styles['pose-button']} ${styles['pose-option']} ${trajectory.id === currentTrajectoryId ? styles['pose-selected'] : ''}`}
      onClick={() => {
        setCurrentTrajectoryId(trajectory.id);
      }}
    >
      <div className={styles['pose-name']}>{trajectory.name}</div>
      <div
        className={`${styles['pose-indicator']} ${
          isCloseEnough(
            predefinedArmTrajectories.START_JOINTS[
              predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[i].name
            ],
            namesAndValues.map((joint) => joint.value),
            predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad,
            [1, 2, 3, 4, 5, 6]
          )
            ? styles['pose-ready']
            : styles['pose-not-ready']
        }`}
      />
    </div>
  ));

  let isJointClose: boolean[] = Array(6).fill(false);

  isJointClose = isJointClose.map((_, i) =>
    namesAndValues.length
      ? Math.abs(
          predefinedArmTrajectories.START_JOINTS[
            predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[currentTrajectoryId].name
          ][i] - namesAndValues[i].value
        ) <= predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad
      : false
  );

  const trajectoryJoints = poseJoints(
    Array.from({ length: 6 }, (_, i) => (
      <div className={`${styles['joint-value']} ${!isJointClose[i] ? styles['warn'] : ''}`} key={i}>
        {rad2deg(predefinedJointValues[i]).toFixed(0)}
      </div>
    ))
  );

  const closeEnough = isCloseEnough(
    predefinedArmTrajectories.START_JOINTS[
      predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[currentTrajectoryId].name
    ],
    namesAndValues.map((joint) => joint.value),
    predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad,
    [1, 2, 3, 4, 5, 6]
  );
  return (
    <div className={styles['pose-requester']}>
      <h2 className={styles['pose-header']}>Trajectory Requester</h2>
      <div className={styles['pose-panel']}>
        {trajectoryJoints}
        <div className={styles['pose-options']}>{trajectoriesToSelect}</div>
      </div>
      <div className={styles['pose-buttons']}>
        <div
          className={`${styles['pose-button']} ${styles['pose-send']} ${
            closeEnough ? styles['send-ready'] : styles['send-not-ready']
          }`}
          onClick={() => {
            if (closeEnough) {
              setKeepAlive(true);
              keepAliveTrajectory();
              sendTrajectoryRequest(currentTrajectoryId);
            }
          }}
        >
          {closeEnough ? 'Send Pose' : 'Cannot Send'}
          {/* MARK sending tra */}
        </div>
        <div
          className={`${styles['pose-button']} ${styles['pose-abort']}`}
          onClick={() => {
            abortTrajectory();
          }}
        >
          Abort
        </div>
      </div>
      <div className={styles['pose-status']}>Status: {lastStatusTrajectory}</div>
    </div>
  );
}

function EditPanel({
  pose,
  onChangePose
}: {
  pose: (ArmPose & { isCustom?: boolean }) | null;
  onChangePose: (pose: ArmPose | null) => void;
}) {
  if (!pose || !pose.joints) {
    return (
      <div className={styles['edit-panel']}>
        <h2 className={styles['edit-header']}>Edit Panel</h2>
        <div className={styles['edit-content']}>
          {pose ? 'Predefined pose selected, cannot edit joints' : 'Select a pose to edit'}
        </div>
      </div>
    );
  }
  const isReadOnly = !pose.isCustom;

  const exportToFile = () => {
    if (!pose) return;

    const {isCustom, ...poseData} = pose;

    const dataStr = JSON.stringify(poseData, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);

    const link = document.createElement('a');
    link.href = url;
    link.download = `arm_pose_${pose.name.replace(/\s+/g, '_')}.json`;
    
    document.body.appendChild(link);
    link.click();

    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const handleDelete = () => {
    if (window.confirm(`Are you sure you want to delete "${pose.name}"?`)) {
      const saved = JSON.parse(localStorage.getItem('custom_arm_poses') || '[]');
      const newList = saved.filter((p: ArmPose) => p.id !== pose.id);
      localStorage.setItem('custom_arm_poses', JSON.stringify(newList));
      window.dispatchEvent(new Event('local-poses-update'));
      onChangePose(null);
    }
  };
  
  return (
    <div className={styles['edit-panel']}>
      {/* TODO input import */}
      <h2>{isReadOnly ? 'View Pose:' : 'Edit Pose:'} {pose.name}</h2>
      {isReadOnly ? (
        <p className={styles['warn-text']}>⚠️ Predefined poses cannot be modified.</p>
      ) : (
        <div className={styles['edit-actions']}>
          {/* <Button onClick={handleRename}>Rename</Button> */}
          <Button onClick={handleDelete} className={styles['btn-danger']}>
            <FontAwesomeIcon icon={faTrash} /> Delete
          </Button>
        </div>
      )}
      <Button onClick={exportToFile}>
        <FontAwesomeIcon icon={faUpload} /> Export Pose
      </Button>
      
      {/* TODO editing panel */}
    </div>
  );
}


export default function Arms() {
  const [isEditMode, setIsEditMode] = useState(false);
  const [selectedPose, setSelectedPose] = useState<ArmPose | null>(null);

  const rerender = useCallback(() => {
    setIsEditMode((prev) => prev);
  }, []);

  useEffect(() => {
    const handler = (e: any) => setIsEditMode(e.detail.enabled);
    window.addEventListener('arm-edit-mode', handler);
    return () => {
      window.removeEventListener('arm-edit-mode', handler);
    };
  }, []);

  return (
    <div className={styles['arm-panel']}>
      <ArmStatus editMode={isEditMode} />
      <div className={styles['trajectory-and-pose']}>
        <PoseRequester 
          editMode={isEditMode}
          onSelectPose={(pose) => setSelectedPose(pose)}
        />
        <div className={styles['extra-space']} />
        <TrajectoryRequester />
      </div>
      {isEditMode && (
        <EditPanel
          pose={selectedPose}
          onChangePose={(newPose) => {
            setSelectedPose(newPose);
          }}
        />
      )}
    </div>
  );
}
