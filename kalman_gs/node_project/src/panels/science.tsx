import styles from './science.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { faCircleExclamation, faRefresh } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useCallback, useEffect, useState } from 'react';
import { Service } from 'roslib';

enum ScienceElementType {
  DISPLAY = 0,
  CONTAINER = 1,
  PLAYER = 2
}

type ScienceElements = {
  enable?: boolean;
  science_elements?: ScienceElement[];
};

type ScienceElement = {
  id?: string;
  type?: ScienceElementType;
  display_name?: string;
  buttons?: ScienceButton[];
  color?: string;
};

type ScienceButton = {
  id?: string;
  name?: string;
  color?: string;
};

// Services:
let scienceElementsService: Service<{}, ScienceElements> = null;

window.addEventListener('ros-connect', () => {
  scienceElementsService = new Service({
    ros: ros,
    name: '/science_panel/get_config',
    serviceType: 'kalman_interfaces/GetScienceElements'
  });

  window.dispatchEvent(new Event('able-to-request-science-elements'));
});

export default function Science() {
  const [scienceElements, setScienceElements] = useState(null);

  const requestScienceElements = useCallback(() => {
    if (scienceElementsService !== null) {
      scienceElementsService.callService(
        {},
        (data: ScienceElements) => setScienceElements(data),
        (err) => console.log(err)
      );
    }
  }, [setScienceElements]);

  useEffect(() => {
    requestScienceElements();
    window.addEventListener('able-to-request-science-elements', requestScienceElements);
    return () => {
      window.removeEventListener('able-to-request-science-elements', requestScienceElements);
    };
  }, [requestScienceElements]);

  if (scienceElements === null) {
    return (
      <div className={styles['science']}>
        <FontAwesomeIcon icon={faCircleExclamation} className={styles['science-danger-exclamation-icon']} />
        <h1>Waiting for service...</h1>
      </div>
    );
  }

  if (!scienceElements.enable) {
    return (
      <div className={styles['science']}>
        <FontAwesomeIcon icon={faCircleExclamation} className={styles['science-danger-exclamation-icon']} />
        <h1>Science panel is disabled.</h1>
        <FontAwesomeIcon
          icon={faRefresh}
          className={styles['science-danger-refresh-icon']}
          onClick={requestScienceElements}
        />
      </div>
    );
  }

  return <div className={styles['science']}></div>;
}
