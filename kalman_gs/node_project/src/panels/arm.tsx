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
import {
  armJointsLocks,
  currentAxisLockFocus,
  toggleArmJointLock
} from '../common/gamepad-arming';
import predefinedPoses from '../common/predefined-arm-poses';
import '../common/predefined-arm-trajectories';
import predefinedArmTrajectories from '../common/predefined-arm-trajectories';
import { ros } from '../common/ros';
import { JointState } from '../common/ros-interfaces';
import { faLock, faLockOpen } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useState, useEffect, useCallback } from 'react';
import { Topic } from 'roslib';

let lastJointState: JointState | null = null;

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

  return namesAndValues.slice(1);
}

function isCloseEnough(
  jointsA: number[],
  jointsB: number[],
  maxDistance: number,
  checkedJoints: number[] = []
) {
  if (jointsA.length !== jointsB.length) {
    return false;
  }
  for (let i = 0; i < jointsA.length; i++) {
    if (
      i + 1 in checkedJoints &&
      Math.abs(jointsA[i] - jointsB[i]) > maxDistance
    ) {
      return false;
    }
  }
  return true;
}

function ArmStatus() {
  const [rerenderCount, setRerenderCount] = useState(0);
  const [linearScale, setLinearScale] = useState<number | null>(
    lastServoLinearScale
  );
  const [rotationalScale, setRotationalScale] = useState<number | null>(
    lastServoRotationalScale
  );

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

  const handleRotationalScaleChange = (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
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
    return Math.min(
      joint.value - jointLimitsRad[i].min,
      jointLimitsRad[i].max - joint.value
    );
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
    <div
      className={
        (stylesFromLimits[i] ? stylesFromLimits[i] : '') + styles['joint-value']
      }
      key={i}
    >
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
      {armJointsLocks[`joint_${i + 1}`] ? (
        <FontAwesomeIcon icon={faLock} />
      ) : (
        <FontAwesomeIcon icon={faLockOpen} />
      )}
    </div>
  ));

  const getAxisLockIcon = (axis: string) => (
    <div
      className={`${styles['joint-lock']}`}
      key={axis + armAxesLocks[axis]}
      onClick={() => toggleArmAxisLock(axis)}
    >
      {armAxesLocks[axis] ? (
        <FontAwesomeIcon icon={faLock} />
      ) : (
        <FontAwesomeIcon icon={faLockOpen} />
      )}
    </div>
  );

  return (
    <div className={styles['arm-status']}>
      <h1 className={styles['status-header']}>Arm Status</h1>
      <div className={styles['status']}>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>
          {jointLocks}
        </div>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>
          {jointNames}
        </div>
        <div className={styles['joint-column'] + ' ' + styles['align-left']}>
          {jointRangeLeft}
        </div>
        <div className={styles['joint-column']}>{jointValues}</div>
        <div className={styles['joint-column'] + ' ' + styles['align-right']}>
          {jointRangeRight}
        </div>
      </div>
      <h3 className={styles['scales-header']}>Scales</h3>
      <div className={styles['scales']}>
        <div className={styles['scale-column'] + ' ' + styles['align-left']}>
          <div className={styles['scale-name']}>Linear scale:</div>
          <div className={styles['scale-name']}>Rotational scale:</div>
        </div>
        <div className={styles['scale-column']}>
          <div
            className={styles['scale-value-holder']}
            key={lastServoLinearScale}
          >
            <input
              type='range'
              min='0'
              max='1'
              step='0.01'
              value={linearScale || 0}
              onChange={handleLinearScaleChange}
            />
            <div className={styles['scale-value'] + ' ' + styles['blink']}>
              {linearScale?.toFixed(2)}
            </div>
          </div>

          <div
            className={styles['scale-value-holder']}
            key={lastServoRotationalScale + 10}
          >
            <input
              type='range'
              min='0'
              max='1'
              step='0.01'
              value={rotationalScale || 0}
              onChange={handleRotationalScaleChange}
            />
            <div className={styles['scale-value'] + ' ' + styles['blink']}>
              {rotationalScale?.toFixed(2)}
            </div>
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
      <div className={styles['joint-column'] + ' ' + styles['align-left']}>
        {getJointNames()}
      </div>
      <div className={styles['joint-column']}>{jointValues}</div>
    </div>
  );
}

function PoseRequester() {
  const [rerenderCount, setRerenderCount] = useState(0);
  const rerender = useCallback(() => {
    setRerenderCount((count) => count + 1);
  }, [setRerenderCount]);

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
      keepAlivePose();
    }, 200);

    // Cleanup interval on component unmount
    return () => clearInterval(intervalId);
  }, []);

  const isStartingFromSafePose = (safePreviousPoses: number[]) => {
    for (let i = 0; i < safePreviousPoses.length; i++) {
      if (
        isCloseEnough(
          predefinedPoses.POSES_JOINTS[
            predefinedPoses.PREDEFINED_POSES.poses[safePreviousPoses[i]].name
          ],
          namesAndValues.map((joint) => joint.value),
          predefinedPoses.PREDEFINED_POSES.max_distance_rad,
          predefinedPoses.PREDEFINED_POSES.poses[safePreviousPoses[i]]
            .joints_checked
        )
      ) {
        return true;
      }
    }
    return false;
  };

  const [currentPoseId, setCurrentPoseId] = useState(0);

  const namesAndValues = getNamesAndValues();

  const predefinedJointValues =
    predefinedPoses.POSES_JOINTS[
      predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].name
    ];

  const posesToSelect = predefinedPoses.PREDEFINED_POSES.poses.map(
    (pose, i) => (
      <div
        key={pose.id}
        className={`${styles['pose-button']} ${styles['pose-option']} ${pose.id === currentPoseId ? styles['pose-selected'] : ''}`}
        onClick={() => {
          setCurrentPoseId(pose.id);
        }}
      >
        <div className={styles['pose-name']}>{pose.name}</div>
        <div
          className={`${styles['pose-indicator']} ${
            isCloseEnough(
              predefinedPoses.POSES_JOINTS[
                predefinedPoses.PREDEFINED_POSES.poses[i].name
              ],
              namesAndValues.map((joint) => joint.value),
              predefinedPoses.PREDEFINED_POSES.max_distance_rad,
              predefinedPoses.PREDEFINED_POSES.poses[i].joints_checked
            ) || isStartingFromSafePose(pose.safe_previous_poses)
              ? styles['pose-ready']
              : styles['pose-not-ready']
          }`}
        />
      </div>
    )
  );

  let isJointSet: boolean[] = Array(6).fill(false);
  let isJointClose: boolean[] = Array(6).fill(false);
  let isJointChecked: boolean[] = Array(6).fill(false);

  predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].joints_set.forEach(
    (value) => {
      isJointSet[value - 1] = true;
    }
  );

  isJointClose = isJointClose.map((_, i) =>
    namesAndValues.length
      ? Math.abs(
          predefinedPoses.POSES_JOINTS[
            predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].name
          ][i] - namesAndValues[i].value
        ) <= predefinedPoses.PREDEFINED_POSES.max_distance_rad
      : false
  );

  predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].joints_checked.forEach(
    (value) => {
      isJointChecked[value - 1] = true;
    }
  );

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

  const closeEnough =
    isCloseEnough(
      predefinedPoses.POSES_JOINTS[
        predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].name
      ],
      namesAndValues.map((joint) => joint.value),
      predefinedPoses.PREDEFINED_POSES.max_distance_rad,
      predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].joints_checked
    ) ||
    isStartingFromSafePose(
      predefinedPoses.PREDEFINED_POSES.poses[currentPoseId].safe_previous_poses
    );
  return (
    <div className={styles['pose-requester']}>
      <h2 className={styles['pose-header']}>Pose Requester</h2>
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
              sendPoseRequest(currentPoseId);
            }
          }}
        >
          {closeEnough ? 'Send Pose' : 'Cannot Send'}
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

function TrajectoryRequester() {
  const [rerenderCount, setRerenderCount] = useState(0);
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
      keepAliveTrajectory();
    }, 200);

    // Cleanup interval on component unmount
    return () => clearInterval(intervalId);
  }, []);

  const [currentTrajectoryId, setCurrentTrajectoryId] = useState(0);

  const namesAndValues = getNamesAndValues();

  const predefinedJointValues =
    predefinedArmTrajectories.START_JOINTS[
      predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[
        currentTrajectoryId
      ].name
    ];

  const trajectoriesToSelect =
    predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories.map(
      (trajectory, i) => (
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
                  predefinedArmTrajectories.PREDEFINED_TRAJECTORIES
                    .trajectories[i].name
                ],
                namesAndValues.map((joint) => joint.value),
                predefinedArmTrajectories.PREDEFINED_TRAJECTORIES
                  .max_distance_rad,
                [1, 2, 3, 4, 5, 6]
              )
                ? styles['pose-ready']
                : styles['pose-not-ready']
            }`}
          />
        </div>
      )
    );

  let isJointClose: boolean[] = Array(6).fill(false);

  isJointClose = isJointClose.map((_, i) =>
    namesAndValues.length
      ? Math.abs(
          predefinedArmTrajectories.START_JOINTS[
            predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[
              currentTrajectoryId
            ].name
          ][i] - namesAndValues[i].value
        ) <= predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad
      : false
  );

  const trajectoryJoints = poseJoints(
    Array.from({ length: 6 }, (_, i) => (
      <div
        className={`${styles['joint-value']} ${!isJointClose[i] ? styles['warn'] : ''}`}
        key={i}
      >
        {rad2deg(predefinedJointValues[i]).toFixed(0)}
      </div>
    ))
  );

  const closeEnough = isCloseEnough(
    predefinedArmTrajectories.START_JOINTS[
      predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[
        currentTrajectoryId
      ].name
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
              sendTrajectoryRequest(currentTrajectoryId);
            }
          }}
        >
          {closeEnough ? 'Send Pose' : 'Cannot Send'}
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
      <div className={styles['pose-status']}>
        Status: {lastStatusTrajectory}
      </div>
    </div>
  );
}
export default function Arms() {
  return (
    <div className={styles['arm-panel']}>
      <ArmStatus />
      <div className={styles['trajectory-and-pose']}>
        <PoseRequester />
        <div className={styles['extra-space']} />
        <TrajectoryRequester />
      </div>
    </div>
  );
}
