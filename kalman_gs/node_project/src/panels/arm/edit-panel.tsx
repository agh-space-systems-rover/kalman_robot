import React from 'react';
import styles from '../arm.module.css';
import { ArmPose, CUSTOM_POSES_KEY, getNamesAndValues } from '../../common/arm';
import { modalRef } from '../../common/refs';
import { faSave, faTrash, faUpload } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

interface EditPanelProps {
    pose: ArmPose | null;
    onChangePose: (pose: ArmPose | null) => void;
}

export function EditPanel({ pose, onChangePose }: EditPanelProps) {
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

    const updatePoseField = (field: keyof ArmPose, value: any) => {
        if (!pose || !pose.isCustom) return;

        const saved: ArmPose[] = JSON.parse(localStorage.getItem(CUSTOM_POSES_KEY) || '[]');
        const poseIndex = saved.findIndex((p) => p.id === pose.id);

        if (poseIndex !== -1) {
        const updatedPose = { ...saved[poseIndex], [field]: value };
        saved[poseIndex] = updatedPose;

        localStorage.setItem(CUSTOM_POSES_KEY, JSON.stringify(saved));
        window.dispatchEvent(new Event('local-poses-update'));
        onChangePose({ ...pose, [field]: value });
        }
    };

    const exportToFile = () => {
        if (!pose) return;
        const { isCustom, ...poseData } = pose;

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
        modalRef.current?.showConfirm({
        title: 'Delete pose',
        icon: faTrash,
        message: `Are you sure you want to delete "${pose.name}"?`,
        confirmText: 'Delete',
        cancelText: 'Cancel',
        onConfirm: () => {
            const saved: ArmPose[] = JSON.parse(localStorage.getItem(CUSTOM_POSES_KEY) || '[]');
            const newList = saved.filter((p) => p.id !== pose.id);
            localStorage.setItem(CUSTOM_POSES_KEY, JSON.stringify(newList));
            window.dispatchEvent(new Event('local-poses-update'));
            onChangePose(null);
        }
        });
    };

    const handleUpdatePose = () => {
        modalRef.current?.showConfirm({
        title: 'Update Pose Position',
        icon: faSave,
        message: 'Are you sure?',
        confirmText: 'Update',
        cancelText: 'Cancel',
        onConfirm: () => {
            if (!pose || !pose.isCustom) return;
            const namesAndValues = getNamesAndValues();
            const currentJointValues = namesAndValues.map((joint) => +joint.value.toFixed(3));
            updatePoseField('joints', currentJointValues);
        }
        });
    };

    const handleRename = () => {
        modalRef.current?.showPrompt({
        title: 'Rename pose',
        icon: faSave,
        message: 'Enter new name for the pose:',
        defaultValue: pose.name,
        confirmText: 'Save',
        onSubmit: (newName) => {
            if (!newName) return;
            updatePoseField('name', newName);
        }
        });
    };

    const toggleInArray = (field: 'joints_set' | 'joints_checked' | 'joints_reversed', jointIdx: number) => {
        const currentArray = pose[field] || [];
        const jointNum = jointIdx + 1;
        const newArray = currentArray.includes(jointNum)
        ? currentArray.filter((n) => n !== jointNum)
        : [...currentArray, jointNum].sort((a, b) => a - b);

        updatePoseField(field, newArray);
    };

    const JointCheckbox = ({ data, name }: { data: number[]; name: 'joints_set' | 'joints_checked' | 'joints_reversed' }) => (
        <>
        {Array.from({ length: 6 }, (_, i) => (
            <input
            key={i}
            type="checkbox"
            disabled={isReadOnly}
            checked={data.includes(i + 1)}
            onChange={() => toggleInArray(name, i)}
            />
        ))}
        </>
    );

    return (
        <div className={styles['edit-panel']}>
        <h2 className={styles['pose-header']}>{isReadOnly ? 'View Pose' : 'Edit Pose'}</h2>
        <div onClick={exportToFile} className={styles['edit-button']}>
            <FontAwesomeIcon icon={faUpload} /> Export Pose
        </div>

        <div>
            <div className={styles['grid-row']}>
            <div className={styles['row-label']}>Joint</div>
            {Array.from({ length: 6 }, (_, i) => (
                <div key={i} className={styles['column-header']}>
                #{i + 1}
                </div>
            ))}
            </div>
            <div className={styles['grid-row']}>
            <div className={styles['row-label']}>Set</div>
            <JointCheckbox data={pose.joints_set} name="joints_set" />
            </div>
            <div className={styles['grid-row']}>
            <div className={styles['row-label']}>Checked</div>
            <JointCheckbox data={pose.joints_checked} name="joints_checked" />
            </div>
            <div className={styles['grid-row']}>
            <div className={styles['row-label']}>Reversed</div>
            <JointCheckbox data={pose.joints_reversed || []} name="joints_reversed" />
            </div>
        </div>

        <p className={styles['warn']} style={{ visibility: isReadOnly ? 'visible' : 'hidden' }}>
            ⚠️ Predefined poses cannot be modified.
        </p>

        <div
            className={styles['edit-actions']}
            style={{ opacity: isReadOnly ? 0.3 : 1, pointerEvents: isReadOnly ? 'none' : 'auto' }}
        >
            <div onClick={handleRename} className={styles['edit-button']}>
            Rename
            </div>
            <div onClick={handleUpdatePose} className={styles['edit-button']}>
            <FontAwesomeIcon icon={faSave} /> Update
            </div>
            <div onClick={handleDelete} className={styles['edit-button']}>
            <FontAwesomeIcon icon={faTrash} /> Delete
            </div>
        </div>
        </div>
    );
}