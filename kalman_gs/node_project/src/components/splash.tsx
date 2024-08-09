import styles from './splash.module.css';

import logo from '../media/logo.png';
import splash from '../media/splash.jpg';
import { faCamera } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component } from 'react';

// Check the local storage whether the splash screen was already shown.
// If not, show the splash screen on boot.
const splashShown = localStorage.getItem('splashShown');
if (!splashShown) {
  localStorage.setItem('splashShown', 'true');
}

type State = {
  shown: boolean;
};

export default class Splash extends Component<{}, State> {
  static defaultState: State = {
    shown: !splashShown
  };
  state = Splash.defaultState;

  escHandler = (e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      this.hide();
    }
  };

  show() {
    this.setState({ shown: true });
    window.addEventListener('keydown', this.escHandler);
  }

  hide() {
    this.setState({ shown: false });
    window.removeEventListener('keydown', this.escHandler);
  }

  render() {
    return (
      <div
        className={
          styles['splash-bg'] + (this.state.shown ? ` ${styles['shown']}` : '')
        }
        onClick={() => this.hide()}
      >
        <div className={styles['splash']}>
          <div className={styles['splash-header']}>
            <img
              src={splash}
              className={styles['splash-header-image']}
              draggable='false'
            />
            <div className={styles['splash-header-image-vignette']} />
            <div className={styles['splash-header-image-attribution']}>
              <FontAwesomeIcon icon={faCamera} />
              <div className={styles['splash-header-image-attribution-text']}>
                KSAF 2023
              </div>
            </div>
            <img
              src={logo}
              className={styles['splash-header-logo']}
              draggable='false'
            />
          </div>
          <div className={styles['splash-content']}>
            <h1>Kalman Ground Station</h1>
            <p>
              Originally created by Rafał Żelazko @ AGH Space Systems
              <br />
              and open sourced under the MIT license.
            </p>
            <p>
              UI inspired by Blender 3.0,
              <br />
              powered by React, Babylon and Leaflet.
            </p>
          </div>
        </div>
      </div>
    );
  }
}
