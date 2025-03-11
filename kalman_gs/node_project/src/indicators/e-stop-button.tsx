import styles from './e-stop-button.module.css';

import { getKeybind } from '../common/keybinds';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { SetBoolRequest, SetBoolResponse } from '../common/ros-interfaces';
import { faStopCircle, faStop, faPlay } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Service } from 'roslib';

import ContextMenu from '../components/context-menu';

let srv: Service<SetBoolRequest, SetBoolResponse> = null;
window.addEventListener('ros-connect', () => {
  srv = new Service<SetBoolRequest, SetBoolResponse>({
    ros: ros,
    name: '/set_estop',
    serviceType: 'std_srvs/SetBool'
  });

  // const eStopTopic = new Topic({
  //   ros: ros,
  //   name: '/estop_state',
  //   messageType: 'std_msgs/Bool'
  // });
  // eStopTopic.subscribe((msg: Bool) => {
  //
  // });
});

function sendEStopRequest(status: boolean) {
  if (srv === null) {
    alertsRef.current?.pushAlert('Not connected to local ROS instance. Unable to send E-STOP request.', 'error');
    return;
  }

  const req: SetBoolRequest = { data: status };
  srv.callService(
    req,
    (response: SetBoolResponse) => {
      if (response.success) {
        alertsRef.current?.pushAlert(
          `Successfully updated E-STOP status. Currently: ${status ? 'ON' : 'OFF'}.`,
          'success'
        );
      } else {
        alertsRef.current?.pushAlert(`Failed to update E-STOP status. ${response.message}.`, 'error');
      }
    },
    (e) => {
      alertsRef.current?.pushAlert(`Failed to update E-STOP status. ${e}.`, 'error');
    }
  );
}

window.addEventListener('keydown', (event) => {
  if (event.repeat) {
    return;
  }

  if (event.code === getKeybind('Engage Remote E-STOP')) {
    sendEStopRequest(true);
  }
});

export default function EStopButton() {
  return (
    <div onClick={() => sendEStopRequest(true)}>
      <ContextMenu
        items={[
          {
            icon: faStop,
            text: 'Engage',
            onClick: () => sendEStopRequest(true)
          },
          {
            icon: faPlay,
            text: 'Release',
            onClick: () => sendEStopRequest(false)
          }
        ]}
        className={styles['estop-button']}
        tooltip='Remote E-STOP button.\nClick to engage,\nRMB for more options.'
      >
        <FontAwesomeIcon icon={faStopCircle} />
      </ContextMenu>
    </div>
  );
}
