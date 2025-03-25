import styles from './science.module.css';

import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import {
  faBox,
  faBoxOpen,
  faCircleExclamation,
  faRefresh,
  faPlay,
  faStop,
  faPause,
  faCircle
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import React, { useCallback, useEffect, useState } from 'react';
import { Service, Topic } from 'roslib';

import Button from '../components/button';
import Input from '../components/input';

enum ScienceElementType {
  DISPLAY = 0,
  CONTAINER = 1,
  PLAYER = 2,
  STATUS = 3
}

type ScienceElementProps = {
  element: ScienceElement;
};

type ScienceButtonsProps = {
  parent_id: string;
  buttons: ScienceButton[];
};

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

function Display({ element }: ScienceElementProps) {
  {
    element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>;
  }

  return (
    <div className={styles['science-element']}>
      {element.buttons && <Buttons parent_id={element.id} buttons={element.buttons} />}
      <div className={styles['row']}>{/*<Input dis*/}</div>
    </div>
  );
}

function Container({ element }: ScienceElementProps) {
  const openTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_open_container`,
    messageType: 'std_msgs/Empty'
  });

  const closeTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_close_container`,
    messageType: 'std_msgs/Empty'
  });

  const styleSheet = getComputedStyle(document.body);
  const containerStyle = element.color ? { color: styleSheet.getPropertyValue(`--${element.color}`) } : {};

  return (
    <div className={styles['science-element']}>
      {element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>}
      {element.buttons && <Buttons parent_id={element.id} buttons={element.buttons} />}
      <div>
        <FontAwesomeIcon
          icon={faBoxOpen}
          onClick={() => openTopic.publish({})}
          style={containerStyle}
          className={styles['science-element-icon']}
        />
        <FontAwesomeIcon
          icon={faBox}
          onClick={() => closeTopic.publish({})}
          style={containerStyle}
          className={styles['science-element-icon']}
        />
      </div>
    </div>
  );
}

function Player({ element }: ScienceElementProps) {
  const playTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_player_play`,
    messageType: 'std_msgs/Empty'
  });

  const stopTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_player_stop`,
    messageType: 'std_msgs/Empty'
  });

  const pauseTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_player_pause`,
    messageType: 'std_msgs/Empty'
  });

  const styleSheet = getComputedStyle(document.body);
  const playerStyle = element.color ? { color: styleSheet.getPropertyValue(`--${element.color}`) } : {};

  return (
    <div className={styles['science-element']}>
      {element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>}
      {element.buttons && <Buttons parent_id={element.id} buttons={element.buttons} />}
      <div>
        <FontAwesomeIcon
          icon={faPlay}
          onClick={() => playTopic.publish({})}
          style={playerStyle}
          className={styles['science-element-icon']}
        />
        <FontAwesomeIcon
          icon={faStop}
          onClick={() => stopTopic.publish({})}
          style={playerStyle}
          className={styles['science-element-icon']}
        />
        <FontAwesomeIcon
          icon={faPause}
          onClick={() => pauseTopic.publish({})}
          style={playerStyle}
          className={styles['science-element-icon']}
        />
      </div>
    </div>
  );
}

function Status({ element }: ScienceElementProps) {
  const statusTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_status`,
    messageType: 'std_msgs/Bool'
  });

  const [status, setStatus] = useState<boolean | null>(null);
  const [timestamp, setTimestamp] = useState<string>('');

  const styleSheet = getComputedStyle(document.body);
  const defaultColor = styleSheet.getPropertyValue('--dark-background') || '#222';
  const greenColor = 'rgb(102, 178, 51)';
  const redColor = 'rgb(255, 102, 102)';

  let iconColor = defaultColor;
  if (status !== null) {
    iconColor = status ? greenColor : redColor;
  }

  useEffect(() => {
    const callback = (msg: { data: boolean }) => {
      setStatus(msg.data);
      const now = new Date();
      setTimestamp(now.toLocaleTimeString()); // Format: HH:MM:SS
    };

    statusTopic.subscribe(callback);
    return () => {
      statusTopic.unsubscribe(callback);
    };
  }, [element.id]);

  return (
    <div className={styles['science-element']}>
      {element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>}
      <div className={styles['science-element-icon-container']}>
        <FontAwesomeIcon icon={faCircle} style={{ color: iconColor }} className={styles['science-element-icon']} />
        {timestamp && <p className={styles['science-element-icon-text']}>{timestamp}</p>}
      </div>
    </div>
  );
}

function Buttons({ parent_id, buttons }: ScienceButtonsProps) {
  const style = getComputedStyle(document.body);

  return (
    <div className={styles['row']}>
      {buttons.map((button) => {
        let buttonStyle = {};
        if (button.color) buttonStyle = { backgroundColor: style.getPropertyValue(`--${button.color}`) };

        const topic = new Topic({
          ros: ros,
          name: `/science_panel/${parent_id}_${button.id}`,
          messageType: 'std_msgs/Empty'
        });

        return (
          <Button key={button.id} style={buttonStyle} onClick={() => topic.publish({})}>
            {button.name}
          </Button>
        );
      })}
    </div>
  );
}

export default function Science() {
  const [scienceElements, setScienceElements] = useState<ScienceElements | null>(null);

  const handleScienceElements = useCallback((data: ScienceElements) => {
    setScienceElements(data);
  }, []);

  const requestScienceElements = useCallback(() => {
    if (scienceElementsService !== null) {
      scienceElementsService.callService({}, (data: ScienceElements) => handleScienceElements(data), undefined);
    }
  }, [handleScienceElements]);

  useEffect(() => {
    requestScienceElements();
    window.addEventListener('able-to-request-science-elements', requestScienceElements);
    return () => {
      window.removeEventListener('able-to-request-science-elements', requestScienceElements);
    };
  }, [requestScienceElements]);

  if (scienceElements === null) {
    return (
      <div className={styles['science-danger']}>
        <FontAwesomeIcon icon={faCircleExclamation} className={styles['science-danger-exclamation-icon']} />
        <h1>Waiting for service...</h1>
        <FontAwesomeIcon
          icon={faRefresh}
          className={styles['science-danger-refresh-icon']}
          onClick={requestScienceElements}
        />
      </div>
    );
  }

  if (!scienceElements.enable) {
    return (
      <div className={styles['science-danger']}>
        <FontAwesomeIcon icon={faCircleExclamation} className={styles['science-danger-exclamation-icon']} />
        <h1>Science panel is disabled.</h1>
      </div>
    );
  }

  return (
    <div className={styles['science']}>
      {scienceElements.science_elements?.map((elem) => {
        switch (elem.type) {
          case ScienceElementType.DISPLAY:
            return <Display key={elem.id} element={elem} />;
          case ScienceElementType.CONTAINER:
            return <Container key={elem.id} element={elem} />;
          case ScienceElementType.PLAYER:
            return <Player key={elem.id} element={elem} />;
          case ScienceElementType.STATUS:
            return <Status key={elem.id} element={elem} />;
        }
      })}
    </div>
  );
}
