import styles from './refresh-button.module.css';

import { faRotateRight } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import ContextMenu from '../components/context-menu';

function refreshWindow() {
  window.location.reload();
}

export default function RefreshButton() {
  return (
    <div onClick={refreshWindow}>
      <ContextMenu
        items={[
          {
            icon: faRotateRight,
            text: 'Refresh',
            onClick: refreshWindow
          }
        ]}
        className={styles['refresh-button']}
        tooltip='Refresh window'
      >
        <FontAwesomeIcon icon={faRotateRight} />
      </ContextMenu>
    </div>
  );
}
