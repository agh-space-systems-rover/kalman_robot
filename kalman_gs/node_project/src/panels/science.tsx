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
import React, { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { Service, Topic } from 'roslib';

import Button from '../components/button';

enum ScienceElementType {
  DISPLAY = 0,
  CONTAINER = 1,
  PLAYER = 2,
  STATUS = 3,
  NONE = 4
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
  const displayTopic = new Topic({
    ros: ros,
    name: `/science_panel/${element.id}_display`,
    messageType: 'std_msgs/String'
  });

  const [display, setDisplay] = useState<string>('Waiting for data...');
  const [timestamp, setTimestamp] = useState<string>('Waiting for data...');

  useEffect(() => {
    const callback = (msg: { data: string }) => {
      setDisplay(msg.data);
      const now = new Date();
      setTimestamp(now.toLocaleTimeString()); // Format: HH:MM:SS
    };

    displayTopic.subscribe(callback);
    return () => {
      displayTopic.unsubscribe(callback);
    };
  }, [element.id]);

  return (
    <div className={styles['science-element']}>
      {element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>}
      {element.buttons && <Buttons parent_id={element.id} buttons={element.buttons} />}
      <div className={styles['science-element-display-container']}>
        <div className={styles['science-element-display-data']}>
          <span>{display}</span>
        </div>
        <div className={styles['science-element-display-timestamp']}>
          <span>{timestamp}</span>
        </div>
      </div>
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
      <div className={styles['science-element-icon-container']}>
        <FontAwesomeIcon
          icon={faBoxOpen}
          onClick={() => openTopic.publish({})}
          className={styles['science-element-icon']}
          style={{
            ...containerStyle,
            width: '45px',
            height: '45px'
          }}
        />
        <FontAwesomeIcon
          icon={faBox}
          onClick={() => closeTopic.publish({})}
          className={styles['science-element-icon']}
          style={{
            ...containerStyle,
            width: '42px',
            height: '42px'
          }}
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
      <div className={styles['science-element-icon-container']}>
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
  const [timestamp, setTimestamp] = useState<string>('Waiting for data...');

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

function None({ element }: ScienceElementProps) {
  return (
    <div className={styles['science-element']}>
      {element.display_name && <h2 className={styles['science-element-header']}>{element.display_name}</h2>}
      {element.buttons && <Buttons parent_id={element.id} buttons={element.buttons} />}
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
  const sciencePanel = useRef(null);
  const scienceContainer = useRef(null);
  const [scienceElements, setScienceElements] = useState<ScienceElements | null>(null);

  const handleScienceElements = useCallback((data: ScienceElements) => {
    setScienceElements(data);
  }, []);

  const requestScienceElements = useCallback(() => {
    if (scienceElementsService !== null) {
      scienceElementsService.callService({}, (data: ScienceElements) => handleScienceElements(data), undefined);
    }
  }, [handleScienceElements]);

  const onResize = useCallback(() => {
    if (scienceContainer.current === null) return;

    // Find the heighten child
    const children = Array.from(scienceContainer.current.children) as HTMLElement[];
    let maxChildHeight = 0;
    children.forEach((child) => {
      const childHeight = child.offsetHeight;
      if (childHeight > maxChildHeight) maxChildHeight = childHeight;
    });

    // Calculate optimal height: to handle same number of elements
    let calculatedHeight = maxChildHeight * (Math.round(Math.sqrt(children.length)) + 1);

    scienceContainer.current.style.maxHeight = `${calculatedHeight}px`;
  }, [scienceContainer, scienceElements]);

  useLayoutEffect(() => {
    if (scienceContainer.current !== null) {
      onResize();
    }
  }, [scienceElements]);

  useEffect(() => {
    requestScienceElements();
    window.addEventListener('able-to-request-science-elements', requestScienceElements);
    window.addEventListener('resize', onResize);
    window.addEventListener('any-panel-resize', onResize);
    return () => {
      window.removeEventListener('able-to-request-science-elements', requestScienceElements);
      window.removeEventListener('resize', onResize);
      window.removeEventListener('any-panel-resize', onResize);
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
    <div className={styles['science']} ref={sciencePanel}>
      <h1 className={styles['science-header']}>Science componenets</h1>
      <div className={styles['science-container']} ref={scienceContainer}>
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
            case ScienceElementType.NONE:
              return <None key={elem.id} element={elem} />;
          }
        })}
      </div>
    </div>
  );
}
