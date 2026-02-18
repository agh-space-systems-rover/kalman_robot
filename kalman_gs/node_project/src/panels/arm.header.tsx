import styles from './arm.header.module.css';
import { 
   faSave, 
   faEdit, 
   faDownload, 
   faUpload, 
   faTimes, 
   faTrash 
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useState } from 'react';
import Button from '../components/button';

export default function ArmHeader() {
   const [isEditMode, setIsEditMode] = useState(false);

   const toggleEditMode = () => {
      const newMode = !isEditMode;
      setIsEditMode(newMode);
      
      // sending CustomEvent
      window.dispatchEvent(new CustomEvent('arm-edit-mode', { 
      detail: { enabled: newMode } 
      }));
   };

return (
   <div className={styles['arm-header']}>
      {/* editmode button */}
      <div className={styles['section']}>
         <Button 
            tooltip={isEditMode ? "Exit Edit Mode" : "Enter Edit Mode"}
            onClick={toggleEditMode}
            className={isEditMode ? styles['btn-active'] : ''}
         >
            <FontAwesomeIcon icon={isEditMode ? faTimes : faEdit} />
            <span className={styles['btn-text']}>
               {isEditMode ? "Finish Editing" : "Edit Mode"}
            </span>
         </Button>
         </div>
      </div>
   );
}