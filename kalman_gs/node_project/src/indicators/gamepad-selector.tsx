import styles from './gamepad-selector.module.css';

import { GamepadMode, gamepads, GamepadType, setGamepadMode } from '../common/gamepads';
import {
  faBan,
  faComputerMouse,
  faGamepad,
  faGear,
  faMicrochip,
  faOilWell,
  faProjectDiagram,
  faTriangleExclamation,
  IconDefinition
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useState } from 'react';

import ContextMenu from '../components/context-menu';
import Tooltip from '../components/tooltip';

const MODES = [
  { mode: GamepadMode.Wheels, name: 'Wheels', icon: faGear },
  { mode: GamepadMode.Arm, name: 'Arm', icon: faProjectDiagram },
  { mode: GamepadMode.Drill, name: 'Drill', icon: faOilWell },
  { mode: GamepadMode.None, name: 'Unassigned', icon: faBan }
];

// Devices that are only displayed.
const DEVICES: { [type: number]: { icon: IconDefinition; overlay?: IconDefinition; description: string } } = {
  [GamepadType.Custom]: { icon: faGamepad, overlay: faMicrochip, description: 'Custom controller' },
  [GamepadType.SpaceMouse]: { icon: faComputerMouse, description: 'SpaceMouse' },
  [GamepadType.Unsupported]: {
    icon: faGamepad,
    overlay: faTriangleExclamation,
    description: 'Not supported. If it is a gamepad, switch it to XInput mode.'
  }
};

export default function GamepadSelector() {
  const [_, setRerenderCount] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCount((prev) => prev + 1);
  }, []);

  useEffect(() => {
    window.addEventListener('gamepads-update', rerender);
    return () => window.removeEventListener('gamepads-update', rerender);
  }, [rerender]);

  return (
    <>
      {gamepads.map((pad) => {
        if (pad.type !== GamepadType.Gamepad) {
          const { icon, overlay, description } = DEVICES[pad.type] ?? DEVICES[GamepadType.Unsupported];
          return (
            <Tooltip
              key={pad.id}
              text={`${pad.name}\\n${description}`}
              className={styles['gamepad-selector'] + ' ' + styles['display-only']}
            >
              <FontAwesomeIcon icon={icon} className={styles['gamepad-icon']} />
              {overlay && <FontAwesomeIcon icon={overlay} className={styles['mode-icon']} />}
            </Tooltip>
          );
        }

        return (
          <ContextMenu
            key={pad.id}
            items={MODES.map(({ mode, name, icon }) => ({
              icon,
              text: name,
              onClick: () => setGamepadMode(pad.id, mode)
            }))}
            openOnClick={true}
            className={styles['gamepad-selector']}
            tooltip={pad.name}
          >
            <FontAwesomeIcon icon={faGamepad} className={styles['gamepad-icon']} />
            <FontAwesomeIcon
              icon={MODES.find(({ mode }) => mode === pad.mode)?.icon ?? faBan}
              className={styles['mode-icon']}
            />
          </ContextMenu>
        );
      })}
    </>
  );
}
