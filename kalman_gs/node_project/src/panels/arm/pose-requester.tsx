import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from '../arm.module.css';
import {
    ArmPose,
    CUSTOM_POSES_KEY,
    HIDDEN_POSES_KEY,
    KEEP_ALIVE_STATUSES,
    rad2deg,
    sendPoseRequest,
    abortPose,
    keepAlivePose,
    lastStatusPose
} from '../../common/arm';
import predefinedPoses from '../../common/predefined-arm-poses';
import { modalRef } from '../../common/refs';
import { faDownload, faEye, faEyeSlash } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

interface PoseRequesterProps {
editMode: boolean;
onSelectPose: (pose: ArmPose) => void;
}

export function PoseRequester({ editMode, onSelectPose }: PoseRequesterProps) {
    const [, setRerenderCount] = useState(0);
    const keepAlive = useRef(false);
    const prevLastStatusPoseRef = useRef<string | null>(null);

    const [customPoses, setCustomPoses] = useState<ArmPose[]>([]);
    const [currentPoseId, setCurrentPoseId] = useState<number>(0);

    const [hiddenPoseIds, setHiddenPoseIds] = useState<number[]>(() => {
        const savedHiddenRaw = localStorage.getItem(HIDDEN_POSES_KEY);
        return savedHiddenRaw ? JSON.parse(savedHiddenRaw) : [];
    });

    const rerender = useCallback(() => {
        setRerenderCount((count) => count + 1);
    }, []);

    useEffect(() => {
        const loadCustomPoses = () => {
        const savedPosesRaw = localStorage.getItem(CUSTOM_POSES_KEY);
        const savedPoses: ArmPose[] = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];
        setCustomPoses(savedPoses);
        };

        loadCustomPoses();
        window.addEventListener('local-poses-update', loadCustomPoses);
        return () => window.removeEventListener('local-poses-update', loadCustomPoses);
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
        if (keepAlive.current) {
            keepAlivePose();
        }
        if (prevLastStatusPoseRef.current !== lastStatusPose && !KEEP_ALIVE_STATUSES.includes(lastStatusPose)) {
            keepAlive.current = false;
        }
        prevLastStatusPoseRef.current = lastStatusPose;
        }, 200);

        return () => clearInterval(intervalId);
    }, []);

    const allPoses: ArmPose[] = [
        ...predefinedPoses.poses.map((p) => ({ ...p, isCustom: false })),
        ...customPoses.map((p) => ({ ...p, isCustom: true }))
    ];

    const visiblePoses = editMode ? allPoses : allPoses.filter((pose) => !hiddenPoseIds.includes(pose.id));
    const currentPose = allPoses.find((p) => p.id === currentPoseId) || allPoses[0];
    const predefinedJointValues = currentPose?.joints ?? [0, 0, 0, 0, 0, 0];

    const handleSend = async () => {
        if (!currentPose) return;
        keepAlive.current = true;
        keepAlivePose();
        await new Promise((r) => setTimeout(r, 200));
        sendPoseRequest(currentPose);
    };

    const handleImportSinglePose = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (event) => {
        try {
            const importedData = JSON.parse(event.target?.result as string);
            const importedArray = Array.isArray(importedData) ? importedData : [importedData];
            const savedPosesRaw = localStorage.getItem(CUSTOM_POSES_KEY);
            let existingPoses: ArmPose[] = savedPosesRaw ? JSON.parse(savedPosesRaw) : [];

            importedArray.forEach((newPose) => {
            const poseToAdd: ArmPose = { ...newPose, isCustom: true };
            while (existingPoses.some((p) => p.id === poseToAdd.id)) {
                poseToAdd.id = Date.now();
            }
            if (existingPoses.some((p) => p.name === poseToAdd.name)) {
                poseToAdd.name = `${poseToAdd.name} (Imported)`;
            }
            existingPoses.push(poseToAdd);
            });

            localStorage.setItem(CUSTOM_POSES_KEY, JSON.stringify(existingPoses));
            window.dispatchEvent(new Event('local-poses-update'));
            modalRef.current?.showAlert({
            title: 'Import complete',
            icon: faDownload,
            message: `Successfully imported ${importedArray.length} pose(s).`
            });
        } catch {
            modalRef.current?.showAlert({
            title: 'Import failed',
            icon: faDownload,
            message: 'Invalid JSON format.'
            });
        }
        };
        reader.readAsText(file);
        e.target.value = '';
    };

    return (
        <div className={styles['pose-requester']}>
        <div className={styles['header-container']}>
            <h2 className={styles['pose-header']}>Pose Requester</h2>
            {editMode && (
            <label className={styles['import-label']}>
                <input type="file" accept=".json" style={{ display: 'none' }} onChange={handleImportSinglePose} />
                <div className={styles['joint-lock']} title="Import Pose JSON">
                <FontAwesomeIcon icon={faDownload} />
                </div>
            </label>
            )}
        </div>

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
                <div className={styles['joint-value']} key={i}>
                    {rad2deg(predefinedJointValues[i]).toFixed(0)}
                </div>
                ))}
            </div>
            </div>

            <div className={styles['pose-options']}>
            {visiblePoses.map((pose) => (
                <div
                key={pose.id}
                className={`${styles['pose-button']} ${styles['pose-option']} ${
                    pose.id === currentPoseId ? styles['pose-selected'] : ''
                }`}
                onClick={() => {
                    setCurrentPoseId(pose.id);
                    if (editMode && pose.joints) onSelectPose(pose);
                }}
                >
                <div className={styles['pose-name']}>
                    {pose.name}
                </div>
                <div className={styles['pose-option-actions']}>
                    {editMode && (
                    <button
                        type="button"
                        className={styles['pose-eye-button']}
                        title={hiddenPoseIds.includes(pose.id) ? 'Show pose' : 'Hide pose'}
                        onClick={(e) => {
                        e.stopPropagation();
                        setHiddenPoseIds((prev) => {
                            const next = prev.includes(pose.id) ? prev.filter((id) => id !== pose.id) : [...prev, pose.id];
                            localStorage.setItem(HIDDEN_POSES_KEY, JSON.stringify(next));
                            return next;
                        });
                        }}
                    >
                        <FontAwesomeIcon icon={hiddenPoseIds.includes(pose.id) ? faEyeSlash : faEye} />
                    </button>
                    )}
                    <div className={`${styles['pose-indicator']} ${styles['pose-ready']}`} />
                </div>
                </div>
            ))}
            </div>
        </div>

        <div className={styles['pose-buttons']}>
            <div className={`${styles['pose-button']} ${styles['pose-send']} ${styles['send-ready']}`} onClick={handleSend}>
            Send Pose
            </div>
            <div className={`${styles['pose-button']} ${styles['pose-abort']}`} onClick={abortPose}>
            Abort
            </div>
        </div>
        <div className={styles['pose-status']}>Status: {lastStatusPose}</div>
        </div>
    );
}