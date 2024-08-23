import styles from './gamepad-selector.module.css';

import { faSteam } from '@fortawesome/free-brands-svg-icons';
import {
  faBan,
  faGamepad,
  faWheelchair,
  IconDefinition
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';

import ContextMenu from '../components/context-menu';

export enum GMode {
  WHEELS,
  ARM
}
export let gamepadsMap;

const GModeIcons: { [key in GMode]: IconDefinition } = {
  [GMode.WHEELS]: faWheelchair,
  [GMode.ARM]: faSteam
};

function normalizePadId(id: string) {
  // Remove hex-hex- prefix
  return id.replace(/^[0-9a-f]{4}-[0-9a-f]{4}-/, '');
}

export default function GamepadSelector() {
  const [gamepads, setGamepads] = useState(new Map<Gamepad, GMode>()); // gamepads are keys and their values are GMode

  useEffect(() => {
    const updateGamepads = () => {
      const connectedGamepads = (
        navigator.getGamepads() ? navigator.getGamepads() : []
      ).filter((gp) => gp !== null);

      setGamepads((prevGamepads) => {
        let newMap = new Map();
        connectedGamepads.forEach((val) => {
          newMap.set(
            val,
            prevGamepads.get(val) ? prevGamepads.get(val) : GMode.WHEELS
          );
        });
        gamepadsMap = newMap;
        return newMap;
      });
    };

    window.addEventListener('gamepadconnected', updateGamepads);
    window.addEventListener('gamepaddisconnected', updateGamepads);

    const interval = setInterval(updateGamepads, 666);
    return () => {
      window.removeEventListener('gamepadconnected', updateGamepads);
      window.removeEventListener('gamepaddisconnected', updateGamepads);
      clearInterval(interval);
    };
  }, []);

  return (
    <>
      {Array.from(gamepads.keys()).map((pad) => (
        <ContextMenu
          items={[
            {
              icon: faWheelchair,
              text: 'Wheels',
              onClick: () => {
                setGamepads(new Map(gamepads.set(pad, GMode.WHEELS)));
                gamepadsMap = gamepads;
              }
            },
            {
              icon: faSteam,
              text: 'Arm',
              onClick: () => {
                setGamepads(new Map(gamepads.set(pad, GMode.ARM)));
                gamepadsMap = gamepads;
              }
            }
          ]}
          openOnClick={true}
          className={styles['gamepad-selector']}
          tooltip={normalizePadId(pad.id)}
        >
          <FontAwesomeIcon
            icon={faGamepad} className={styles['gamepad-icon']}
          />
          <FontAwesomeIcon
            icon={GModeIcons[gamepads.get(pad)]} className={styles['mode-icon']}
          />
        </ContextMenu>
      ))}
    </>
  );
}
