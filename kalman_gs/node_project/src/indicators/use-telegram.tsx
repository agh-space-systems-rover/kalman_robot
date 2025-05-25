import styles from './use-telegram.module.css';

import { faTelegram } from '@fortawesome/free-brands-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import Tooltip from '../components/tooltip';


export default function UseTelegram() {
  return (
    <Tooltip
      text='Send Telegram message or image.'
      className={styles['use-telegram']}
      onClick={() => {
        console.log('xyz')
      }}
    >
      <FontAwesomeIcon icon={faTelegram} />
    </Tooltip>
  );
}