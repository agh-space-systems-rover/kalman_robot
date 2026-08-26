import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from '../arm.module.css';
import {
    KEEP_ALIVE_STATUSES,
    rad2deg,
    isCloseEnough,
    getNamesAndValues,
    keepAliveTrajectory,
    sendTrajectoryRequest,
    abortTrajectory,
    lastStatusTrajectory
} from '../../common/arm';
import predefinedArmTrajectories from '../../common/predefined-arm-trajectories';

type TrajectoryName = keyof typeof predefinedArmTrajectories.START_JOINTS;

export function TrajectoryRequester() {
    const [, setRerenderCount] = useState(0);
    const keepAlive = useRef(false);
    const prevLastStatusTrajRef = useRef<string | null>(null);
    const [currentTrajectoryId, setCurrentTrajectoryId] = useState(0);

    const rerender = useCallback(() => {
        setRerenderCount((count) => count + 1);
    }, []);

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
        if (keepAlive.current) {
            keepAliveTrajectory();
        }
        if (prevLastStatusTrajRef.current !== lastStatusTrajectory && !KEEP_ALIVE_STATUSES.includes(lastStatusTrajectory)) {
            keepAlive.current = false;
        }
        prevLastStatusTrajRef.current = lastStatusTrajectory;
        }, 200);

        return () => clearInterval(intervalId);
    }, []);

    const namesAndValues = getNamesAndValues();

    const currentTrajectory = predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories[currentTrajectoryId];
    const trajName = currentTrajectory?.name as TrajectoryName | undefined;
    const predefinedJointValues = (trajName && predefinedArmTrajectories.START_JOINTS[trajName]) || [0, 0, 0, 0, 0, 0];

    let isJointClose: boolean[] = Array(6).fill(false);
    isJointClose = isJointClose.map((_, i) =>
        namesAndValues.length
        ? Math.abs(predefinedJointValues[i] - namesAndValues[i].value) <=
            predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad
        : false
    );

    const closeEnough = isCloseEnough(
        predefinedJointValues,
        namesAndValues.map((joint) => joint.value),
        predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad,
        [1, 2, 3, 4, 5, 6]
    );

    return (
        <div className={styles['pose-requester']}>
        <h2 className={styles['pose-header']}>Trajectory Requester</h2>
        
        <div className={styles['pose-panel']}>
            <div className={styles['status']}>
            <div className={`${styles['joint-column']} ${styles['align-left']}`}>
                {Array.from({ length: 6 }, (_, i) => (
                <div className={styles['joint-name']} key={i}>
                    Joint {i + 1}:
                </div>
                ))}
            </div>
            <div className={styles['joint-column']}>
                {Array.from({ length: 6 }, (_, i) => (
                <div className={`${styles['joint-value']} ${!isJointClose[i] ? styles['warn'] : ''}`} key={i}>
                    {rad2deg(predefinedJointValues[i]).toFixed(0)}
                </div>
                ))}
            </div>
            </div>

            <div className={styles['pose-options']}>
                {predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.trajectories.map((trajectory, i) => {
                    type TrajectoryName = keyof typeof predefinedArmTrajectories.START_JOINTS;
                    const startJoints = predefinedArmTrajectories.START_JOINTS[trajectory.name as TrajectoryName] || [];

                    const isReady = isCloseEnough(
                    startJoints,
                    namesAndValues.map((joint) => joint.value),
                    predefinedArmTrajectories.PREDEFINED_TRAJECTORIES.max_distance_rad,
                    [1, 2, 3, 4, 5, 6]
                    );

                    return (
                    <div
                        key={trajectory.id}
                        className={`${styles['pose-button']} ${styles['pose-option']} ${
                        trajectory.id === currentTrajectoryId ? styles['pose-selected'] : ''
                        }`}
                        onClick={() => setCurrentTrajectoryId(trajectory.id)}
                    >
                        <div className={styles['pose-name']}>{trajectory.name}</div>
                        <div className={styles['pose-option-actions']}>
                        <div
                            className={`${styles['pose-indicator']} ${
                            isReady ? styles['pose-ready'] : styles['pose-not-ready']
                            }`}
                        />
                        </div>
                    </div>
                    );
                })}
            </div>
        </div>

        <div className={styles['pose-buttons']}>
            <div
            className={`${styles['pose-button']} ${styles['pose-send']} ${
                closeEnough ? styles['send-ready'] : styles['send-not-ready']
            }`}
            onClick={async () => {
                if (closeEnough) {
                keepAlive.current = true;
                keepAliveTrajectory();
                await new Promise((r) => setTimeout(r, 500));
                sendTrajectoryRequest(currentTrajectoryId);
                }
            }}
            >
            {closeEnough ? 'Send Trajectory' : 'Cannot Send'}
            </div>
            <div className={`${styles['pose-button']} ${styles['pose-abort']}`} onClick={abortTrajectory}>
            Abort
            </div>
        </div>
        
        <div className={styles['pose-status']}>Status: {lastStatusTrajectory}</div>
        </div>
    );
}