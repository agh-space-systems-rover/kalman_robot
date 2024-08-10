import styles from './open-settings.module.css';

import { settingsRef } from '../common/refs';
import { faCog } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import Tooltip from '../components/tooltip';

export default function OpenSettings() {
  return (
    <Tooltip
      text='Show advanced settings.'
      className={styles['open-settings']}
      onClick={() => {
        settingsRef.current?.show();
      }}
    >
      <FontAwesomeIcon icon={faCog} />
    </Tooltip>
  );
}
