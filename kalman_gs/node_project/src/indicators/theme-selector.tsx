import styles from './theme-selector.module.css';

import { faRaspberryPi } from '@fortawesome/free-brands-svg-icons';
import {
  faCloudMoon,
  faEdit,
  faGlasses,
  faHollyBerry,
  faSun
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import ContextMenu from '../components/context-menu';
import Tooltip from '../components/tooltip';

import { setTheme } from '../modules/themes';

export default function ThemeSelector() {
  return (
    <ContextMenu
      items={[
        {
          icon: faCloudMoon,
          text: 'Dark Mode',
          onClick: () => setTheme('dark')
        },
        {
          icon: faSun,
          text: 'Light Mode',
          onClick: () => setTheme('light')
        },
        {
          icon: faRaspberryPi,
          text: 'Berry Purple',
          onClick: () => setTheme('berry')
        }
      ]}
      openOnClick={true}
      className={styles['theme-selector']}
      tooltip='Select a different color scheme for the UI.'
    >
      <FontAwesomeIcon icon={faGlasses} />
    </ContextMenu>
  );
}
