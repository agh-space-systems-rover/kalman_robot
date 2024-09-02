import styles from './gamepad-selector.module.css';

import { GamepadMode, gamepads, setGamepadMode } from '../common/gamepads';
import {
  faGamepad,
  faProjectDiagram,
  faGear,
  IconDefinition,
  faBan,
  faOilWell
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';

import ContextMenu from '../components/context-menu';

export type ModeAppearance = {
  [mode in GamepadMode]: {
    name: string;
    icon: IconDefinition;
  };
};
const modeAppearance: ModeAppearance = {
  'wheels': {
    name: 'Wheels',
    icon: faGear
  },
  'arm': {
    name: 'Arm',
    icon: faProjectDiagram
  },
  'drill': {
    name: 'Drill',
    icon: faOilWell
  },
  'none': {
    name: 'Unassigned',
    icon: faBan
  }
};

function getPadName(id: string) {
  // Remove hex-hex- prefix (Firefox)
  id = id.replace(/^[0-9a-f]{4}-[0-9a-f]{4}-/, '');
  // Remove (...) suffix (Chromium)
  id = id.replace(/ \(.*\)$/, '');
  return id;
}

export default function GamepadSelector() {
  const [_, setRerenderCount] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCount((prev) => prev + 1);
  }, []);

  useEffect(() => {
    window.addEventListener('gamepads-connect', rerender);
    return () => window.removeEventListener('gamepads-connect', rerender);
  }, [rerender]);

  return (
    <>
      {Array.from(gamepads.keys()).map(pad => (
        <ContextMenu
          key={pad.id}
          items={Object.entries(modeAppearance).map(
            ([mode, { name, icon }]) => ({
              icon,
              text: name,
              onClick: () => {
                setGamepadMode(pad, mode as GamepadMode);
                rerender();
              }
            })
          )}
          openOnClick={true}
          className={styles['gamepad-selector']}
          tooltip={getPadName(pad.id)}
        >
          <FontAwesomeIcon
            icon={faGamepad}
            className={styles['gamepad-icon']}
          />
          <FontAwesomeIcon
            icon={modeAppearance[gamepads.get(pad)].icon}
            className={styles['mode-icon']}
          />
        </ContextMenu>
      ))}
    </>
  );
}
