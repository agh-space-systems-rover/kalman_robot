import styles from './alerts.module.css';

import {
  faBug,
  faCheckCircle,
  faExclamationTriangle
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component } from 'react';

type AlertType = 'error' | 'warning' | 'success';

type Alert = {
  id: number;
  shown: boolean;
  message: string;
  type: AlertType;
};

type State = {
  alerts: Alert[];
};

export default class Alerts extends Component<{}, State> {
  static defaultState: State = {
    alerts: []
  };
  state = Alerts.defaultState;

  private static alertIdCounter = 0;
  private visibleAlertIds: number[] = [];

  render() {
    return (
      <div className={styles['alerts']}>
        {this.state.alerts.reverse().map((alert, i) => {
          const icon =
            alert.type === 'error'
              ? faBug
              : alert.type === 'warning'
                ? faExclamationTriangle
                : faCheckCircle;

          return (
            <div
              key={alert.id}
              className={
                styles['alert'] + (alert.shown ? ' shown ' : ' ') + alert.type
              }
              onClick={() => this.hideAlert(alert.id)}
            >
              <FontAwesomeIcon icon={icon} className={styles['icon']} />
              {alert.message}
              <div />
            </div>
          );
        })}
      </div>
    );
  }

  private hideAlert(id: number): void {
    this.setState((state) => ({
      alerts: state.alerts.map((alert) => {
        if (alert.id === id) {
          return {
            ...alert,
            shown: false
          };
        } else {
          return alert;
        }
      })
    }));

    // Schedule a timeout to wait-out the fade-out animation.
    setTimeout(() => {
      if (this.visibleAlertIds.includes(id)) {
        this.visibleAlertIds = this.visibleAlertIds.filter(
          (visibleId) => visibleId !== id
        );
        this.setState((state) => ({
          alerts: state.alerts.filter((alert) => alert.id !== id)
        }));
      }
    }, 300);
  }

  pushAlert(message: string, type: AlertType = 'error') {
    // Generate unique ID for the alert.
    const id = Alerts.alertIdCounter++;

    // Shift all alerts up and insert the new one.
    this.setState((state) => ({
      alerts: [
        {
          id,
          message,
          type,
          shown: false
        },
        ...state.alerts
      ]
    }));
    this.visibleAlertIds.push(id);

    // Compute the timeout based on the length of the message.
    // The minimum timeout is 5 seconds.
    const msgTimeout = Math.max(5000, message.length * 100);

    // Schedule a timeout to show the alert.
    // If we show it immediately, the fade-in animation won't work.
    setTimeout(() => {
      this.setState((state) => ({
        alerts: state.alerts.map((alert) => {
          if (alert.id === id) {
            return {
              ...alert,
              shown: true
            };
          } else {
            return alert;
          }
        })
      }));

      // Schedule a timeout to hide the alert.
      setTimeout(() => {
        this.hideAlert(id);
      }, msgTimeout);
    }, 100);
  }
}
