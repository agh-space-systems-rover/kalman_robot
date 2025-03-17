import styles from './science.module.css';



import { ros } from '../common/ros';
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
});

export default function Ueuos() {
  const [scienceElements, setScienceElements] = useState(null);

  useEffect(() => {
    setInterval(() => {
      if (scienceElementsService !== null) {
        scienceElementsService.callService({}, (data: ScienceElements) => console.log(data), (err) => console.log(err));

      }
    }, 1000)
  }, []);

  return <div className={styles['science']}></div>;
}
