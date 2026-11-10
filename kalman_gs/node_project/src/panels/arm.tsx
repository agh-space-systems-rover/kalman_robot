import React, { useState, useEffect } from 'react';
import styles from './arm.module.css';
import { ArmPose } from '../common/arm';
import { ArmStatus } from './arm/arm-status';
import { PoseRequester } from './arm/pose-requester';
import { EditPanel } from './arm/edit-panel';

export default function Arms() {
  const [isEditMode, setIsEditMode] = useState(false);
  const [selectedPose, setSelectedPose] = useState<ArmPose | null>(null);

  useEffect(() => {
    const handler = (e: any) => setIsEditMode(e.detail.enabled);
    window.addEventListener('arm-edit-mode', handler);
    return () => window.removeEventListener('arm-edit-mode', handler);
  }, []);

  return (
    <div className={styles['arm-panel']}>
      <div className={styles['arm-content']}>
        <ArmStatus editMode={isEditMode} />
        <div className={styles['trajectory-and-pose']}>
          <PoseRequester editMode={isEditMode} onSelectPose={setSelectedPose} />
        </div>
        {isEditMode && <EditPanel pose={selectedPose} onChangePose={setSelectedPose} />}
      </div>
    </div>
  );
}