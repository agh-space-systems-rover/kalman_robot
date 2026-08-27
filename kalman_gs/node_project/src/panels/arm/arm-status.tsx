import React, { useState, useEffect, useCallback } from 'react';
import styles from '../arm.module.css';
import {
    setLinearScaleTo,
    setRotationalScaleTo,
    lastServoLinearScale,
    lastServoRotationalScale,
    toggleArmAxisLock,
    armAxesLocks,
    jointLimitsRad,
    rad2deg,
    getNamesAndValues,
    lastJointState,
    ArmPose,
    CUSTOM_POSES_KEY,
    WARN_THRESHOLD,
    ERROR_THRESHOLD
} from '../../common/arm';
import { armJointsLocks, currentAxisLockFocus, toggleArmJointLock } from '../../common/gamepad-arming';
import { modalRef } from '../../common/refs';
import { faLock, faLockOpen, faSave } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { ArmAxesLocks } from '../../common/ros-interfaces';

interface ArmStatusProps {
    editMode: boolean;
    onPoseSaved?: () => void;
}

export function ArmStatus({ editMode, onPoseSaved }: ArmStatusProps) {
    const [, setRerenderCount] = useState(0);
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
    }, [rerender]);

    const handleLinearScaleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 0 && value <= 1) {
        setLinearScale(value);
        setLinearScaleTo(value);
        }
    };

    const handleRotationalScaleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 0 && value <= 1) {
        setRotationalScale(value);
        setRotationalScaleTo(value);
        }
    };

    const namesAndValues = getNamesAndValues();

    const jointNames = Array.from({ length: 6 }, (_, i) => (
        <div className={styles['joint-name']} key={i}>
        Joint {i + 1}:
        </div>
    ));

    const jointRangeLeft = jointLimitsRad.map((limit, i) => (
        <div className={styles['joint-range']} key={i}>
        {rad2deg(limit.min).toFixed(0)}..
        </div>
    ));

    const distancesFromLimits = namesAndValues.map((joint, i) => {
        if (i > 5) return 0;
        return Math.min(joint.value - jointLimitsRad[i].min, jointLimitsRad[i].max - joint.value);
    });

    const stylesFromLimits = distancesFromLimits.map((distance) => {
        if (distance < ERROR_THRESHOLD) return styles['error'] + ' ';
        if (distance < WARN_THRESHOLD) return styles['warn'] + ' ';
        return '';
    });

    const jointValues = Array.from({ length: 6 }, (_, i) => (
        <div className={(stylesFromLimits[i] ? stylesFromLimits[i] : '') + styles['joint-value']} key={i}>
        {lastJointState && namesAndValues[i] ? rad2deg(namesAndValues[i].value).toFixed(0) : 'N/A'}
        </div>
    ));

    const jointRangeRight = jointLimitsRad.map((limit, i) => (
        <div className={styles['joint-range']} key={i}>
        ..{rad2deg(limit.max).toFixed(0)}
        </div>
    ));

    type JointLockKey = `joint_${1 | 2 | 3 | 4 | 5 | 6}`;
    const jointLocks = Array.from({ length: 6 }, (_, i) => {
        const jointKey = `joint_${i + 1}` as JointLockKey;
        const isLocked = armJointsLocks[jointKey];

        return (
            <div
            className={`${styles['joint-lock']} ${currentAxisLockFocus === i + 1 ? styles['lock-selected'] : ''}`}
            key={i + (isLocked ? 10 : 0)}
            onClick={() => toggleArmJointLock(jointKey)}
            >
            {isLocked ? <FontAwesomeIcon icon={faLock} /> : <FontAwesomeIcon icon={faLockOpen} />}
            </div>
        );
    });

    type ArmAxis = keyof ArmAxesLocks; // 'x' | 'y' | 'z' | 'roll' | 'pitch' | 'yaw'
    const getAxisLockIcon = (axis: ArmAxis) => (
    <div
        className={styles['joint-lock']}
        key={`${axis}-${armAxesLocks[axis] ? 1 : 0}`}
        onClick={() => toggleArmAxisLock(axis)}
    >
        {armAxesLocks[axis] ? <FontAwesomeIcon icon={faLock} /> : <FontAwesomeIcon icon={faLockOpen} />}
    </div>
    );

    const saveCurrentPose = () => {
        modalRef.current?.showPrompt({
        title: 'Save pose',
        icon: faSave,
        message: 'Give the name of new pose:',
        defaultValue: `Pose ${new Date().toLocaleTimeString()}`,
        confirmText: 'Save',
        onSubmit: (poseName) => {
            if (!poseName) return;

            const currentValues = namesAndValues.map((joint) => +joint.value.toFixed(3));
            const savedPosesRaw = localStorage.getItem(CUSTOM_POSES_KEY);
            const savedPoses: ArmPose[] = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];

            const newPose: ArmPose = {
            id: Date.now(),
            name: poseName,
            joints: currentValues
            };

            localStorage.setItem(CUSTOM_POSES_KEY, JSON.stringify([...savedPoses, newPose]));
            window.dispatchEvent(new Event('local-poses-update'));
            onPoseSaved?.();

            modalRef.current?.showAlert({
            title: 'Pose saved',
            icon: faSave,
            message: `"${poseName}" was saved.`
            });
        }
        });
    };

    return (
        <div className={styles['arm-status']}>
        <div className={styles['header-container']}>
            <h1 className={styles['status-header']}>Arm Status</h1>
            {editMode && (
            <div className={styles['joint-lock']} style={{ border: 'none' }} onClick={saveCurrentPose}>
                <FontAwesomeIcon icon={faSave} />
            </div>
            )}
        </div>

        <div className={styles['status']}>
            <div className={`${styles['joint-column']} ${styles['align-left']}`}>{jointLocks}</div>
            <div className={`${styles['joint-column']} ${styles['align-left']}`}>{jointNames}</div>
            <div className={`${styles['joint-column']} ${styles['align-left']}`}>{jointRangeLeft}</div>
            <div className={styles['joint-column']}>{jointValues}</div>
            <div className={`${styles['joint-column']} ${styles['align-right']}`}>{jointRangeRight}</div>
        </div>

        <h3 className={styles['scales-header']}>Scales</h3>
        <div className={styles['scales']}>
            <div className={`${styles['scale-column']} ${styles['align-left']}`}>
            <div className={styles['scale-name']}>Linear scale:</div>
            <div className={styles['scale-name']}>Rotational scale:</div>
            </div>
            <div className={styles['scale-column']}>
            <div className={styles['scale-value-holder']} key={lastServoLinearScale}>
                <input
                type="range"
                min="0"
                max="1"
                step="0.01"
                value={linearScale ?? 0}
                onChange={handleLinearScaleChange}
                />
                <div className={`${styles['scale-value']} ${styles['blink']}`}>{linearScale?.toFixed(2)}</div>
            </div>

            <div className={styles['scale-value-holder']} key={(lastServoRotationalScale ?? 0) + 10}>
                <input
                type="range"
                min="0"
                max="1"
                step="0.01"
                value={rotationalScale ?? 0}
                onChange={handleRotationalScaleChange}
                />
                <div className={`${styles['scale-value']} ${styles['blink']}`}>{rotationalScale?.toFixed(2)}</div>
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