import styles from './autonomy-statuses.module.css';

import { ros } from '../common/ros';
import { faInfo } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Topic } from 'roslib';

import Tooltip from '../components/tooltip';

let autonomyStatusValue: number | null = null;
window.addEventListener('ros-connect', () => {
  const autonomyTopic = new Topic({
    ros: ros,
    name: '/autonomy_status',
    messageType: ''
  });
});

export default function AutonomyStatuses() {
  return (
    <Tooltip
      text='Show autonomy componenets statuses.'
      className={styles['autonomy-statuses']}
      onClick={() => {
        console.log('xxx');
      }}
    >
      <FontAwesomeIcon icon={faInfo} />
    </Tooltip>
  );
}
