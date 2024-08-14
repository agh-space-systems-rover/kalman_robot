import styles from './settings.module.css';

import {
  keybinds,
  resetAllKeybinds,
  resetKeybind,
  setKeybind
} from '../common/keybinds';
import { Theme, currentTheme, setTheme } from '../common/themes';
import Button from './button';
import Dropdown from './dropdown';
import Input from './input';
import { faRaspberryPi } from '@fortawesome/free-brands-svg-icons';
import {
  faBan,
  faCloudMoon,
  faKeyboard,
  faPalette,
  faRefresh,
  faSun,
  faXmark
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component } from 'react';

function keyCodeToName(code: string) {
  if (code === null) {
    return 'Disabled';
  }
  if (code.startsWith('Key')) {
    return code.slice(3);
  }
  if (code.startsWith('Digit')) {
    return code.slice(5);
  }
  if (code.startsWith('Arrow')) {
    return code.slice(5);
  }
  if (code.startsWith('Numpad')) {
    return 'Num ' + code.slice(5);
  }
  if (code.startsWith('Shift')) {
    return code.slice(5) + ' Shift';
  }
  if (code.startsWith('Control')) {
    return code.slice(7) + ' Ctrl';
  }
  if (code.startsWith('Bracket')) {
    return code.slice(7) + ' Bracket';
  }
  if (code.startsWith('Alt')) {
    return code.slice(3) + ' Alt';
  }
  if (code.startsWith('Page')) {
    return 'Page ' + code.slice(4);
  }
  if (code === 'CapsLock') {
    return 'Caps Lock';
  }
  return code;
}

type State = {
  shown: boolean;
  listeningForNewKeybind: string | null;
  searchTerm: string;
};

export default class Settings extends Component<{}, State> {
  static defaultState: State = {
    shown: false,
    listeningForNewKeybind: null,
    searchTerm: ''
  };
  state = Settings.defaultState;

  escHandler = (e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      this.hide();
    } else {
      // Focus the search input when any other key is pressed.
      // Saves the user a little bit of trouble when they forget to click the search box.
      const input = document.querySelector(
        `.${styles['search-input-parent']} input`
      );
      if (input) {
        (input as HTMLInputElement).focus();
        // NOTE: The HTML input box automagically captures this key press.
      }
    }
  };

  show() {
    this.setState({
      shown: true,
      listeningForNewKeybind: null,
      searchTerm: ''
    });
    window.addEventListener('keydown', this.escHandler);
    // Reset scroll in scrollable-options
    const options = document.querySelector(`.${styles['scrollable-options']}`);
    if (options) {
      options.scrollTop = 0;
    }
  }

  hide() {
    this.setState({ shown: false });
    this.stopListeningForNewKeybind();
    window.removeEventListener('keydown', this.escHandler);
  }

  isShown() {
    return this.state.shown;
  }

  startListeningForNewKeybind(action: string) {
    this.setState({ listeningForNewKeybind: action });
    window.addEventListener('keydown', this.keyListener);
    window.removeEventListener('keydown', this.escHandler);
    window.addEventListener(
      'mousedown',
      this.clickToStopListeningForNewKeybindListener
    );
  }

  stopListeningForNewKeybind() {
    this.setState({ listeningForNewKeybind: null });
    window.removeEventListener('keydown', this.keyListener);
    window.addEventListener('keydown', this.escHandler);
    window.removeEventListener(
      'mousedown',
      this.clickToStopListeningForNewKeybindListener
    );
  }

  keyListener = (e: KeyboardEvent) => {
    if (this.state.listeningForNewKeybind) {
      if (e.code === 'Escape') {
        this.stopListeningForNewKeybind();
        return;
      }
      setKeybind(this.state.listeningForNewKeybind, e.code);
      this.stopListeningForNewKeybind();
    }
  };

  clickToStopListeningForNewKeybindListener = (e: MouseEvent) => {
    if (this.state.listeningForNewKeybind) {
      this.stopListeningForNewKeybind();
    }
  };

  isSearchedFor(content: string) {
    if (this.state.searchTerm === '') {
      return true;
    }
    return content.toLowerCase().includes(this.state.searchTerm.toLowerCase());
  }

  render() {
    const searchedKeybinds = Object.entries(keybinds)
      .map(
        ([action, key]) =>
          this.isSearchedFor(
            action + key + keyCodeToName(key) + 'keybinds'
          ) && (
            <div className={styles['keybind']} key={action}>
              <div className={styles['keybind-name']}>{action}</div>
              <div className={styles['keybind-controls']}>
                <Button
                  className={
                    styles['keybind-value'] +
                    (this.state.listeningForNewKeybind === action
                      ? ` ${styles['keybind-value-listening']}`
                      : '') +
                    (this.state.listeningForNewKeybind === null
                      ? ` ${styles['keybind-value-hoverable']}`
                      : '')
                  }
                  tooltip={
                    this.state.listeningForNewKeybind === null
                      ? 'Change this keybind.'
                      : undefined
                  }
                  onClick={() => {
                    if (this.state.listeningForNewKeybind) {
                      return;
                    }
                    this.startListeningForNewKeybind(action);
                  }}
                >
                  &nbsp;
                  {this.state.listeningForNewKeybind === action
                    ? 'Waiting for key...'
                    : keyCodeToName(key)}
                  &nbsp;
                </Button>
                <Button
                  tooltip='Remove this keybind.'
                  onClick={() => {
                    // if (this.state.listeningForNewKeybind === action) {
                    //   this.stopListeningForNewKeybind();
                    // }
                    // Handled by clickToStopListeningForNewKeybindListener.
                    setKeybind(action, null);
                    this.forceUpdate();
                  }}
                >
                  <FontAwesomeIcon icon={faXmark} />
                </Button>
                <Button
                  tooltip='Reset this keybind.'
                  onClick={() => {
                    // if (this.state.listeningForNewKeybind === action) {
                    //   this.stopListeningForNewKeybind();
                    // }
                    // Handled by clickToStopListeningForNewKeybindListener.
                    resetKeybind(action);
                    this.forceUpdate();
                  }}
                >
                  <FontAwesomeIcon icon={faRefresh} />
                </Button>
              </div>
            </div>
          )
      )
      .filter((e) => e); // filter() removes falsy (not searched for) values

    return (
      <div
        className={
          styles['settings-bg'] +
          (this.state.shown ? ` ${styles['shown']}` : '')
        }
        onClick={() => this.hide()}
      >
        <div
          className={styles['settings']}
          onClick={(e) => e.stopPropagation()}
        >
          <div className={styles['content']}>
            <div className={styles['static-header']}>
              <h1>Settings</h1>
              <div className={styles['search-input-parent']}>
                <Input
                  placeholder='Filter options...'
                  onChange={(v) => this.setState({ searchTerm: v })}
                  key={this.state.shown ? 1 : 0} // Ensure the input is focused and cleared when the settings are shown again.
                  // autoFocus
                  // This is not needed because escHandler already focuses when a key is pressed.
                />
              </div>
            </div>
            <div
              className={styles['scrollable-options']}
              key={this.state.searchTerm}
            >
              {this.isSearchedFor(
                'select color theme light mode dark mode berry purple'
              ) && (
                <>
                  <h2>
                    <FontAwesomeIcon icon={faPalette} />
                    &nbsp;&nbsp;Color Theme
                  </h2>
                  <div className={styles['color-theme-selector']}>
                    <Dropdown
                      tooltip='Change the color theme.'
                      items={[
                        {
                          icon: faCloudMoon,
                          text: 'Dark Mode'
                        },
                        {
                          icon: faSun,
                          text: 'Light Mode'
                        },
                        {
                          icon: faRaspberryPi,
                          text: 'Berry Purple'
                        }
                      ]}
                      onChange={(i) => {
                        setTheme(['dark', 'light', 'berry'][i] as Theme);
                      }}
                      defaultItemIndex={['dark', 'light', 'berry'].indexOf(
                        currentTheme
                      )}
                    />
                  </div>
                </>
              )}
              {(searchedKeybinds.length > 0 ||
                this.isSearchedFor('reset all keybinds')) && (
                <>
                  <h2>
                    <FontAwesomeIcon icon={faKeyboard} />
                    &nbsp;&nbsp;Keybinds
                  </h2>
                  <div className={styles['reset-all-keybinds']}>
                    <Button
                      tooltip='Reset all keybinds to default.'
                      onClick={() => {
                        // this.stopListeningForNewKeybind();
                        // Handled by clickToStopListeningForNewKeybindListener.
                        resetAllKeybinds();
                        this.forceUpdate();
                      }}
                    >
                      <FontAwesomeIcon icon={faRefresh} />
                      &nbsp;&nbsp; Reset All Keybinds
                    </Button>
                  </div>
                </>
              )}
              {searchedKeybinds}
              <div className={styles['no-search-results']}>
                <div className={styles['no-search-results-text']}>
                  <FontAwesomeIcon icon={faBan} />
                  &nbsp;&nbsp;No matching options found.
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }
}
