import styles from './settings.module.css';

import { BackgroundImage, getBackgroundImagesLabel, setBackgroundImages } from '../common/background-images';
import {
  DEFAULT_DRILL_GAMEPAD_SETTINGS,
  DrillGamepadSetting,
  getDrillGamepadSettings,
  resetDrillGamepadSetting,
  setDrillGamepadSetting
} from '../common/gamepad-drilling-settings';
import { gamepads, getGamepadName } from '../common/gamepads';
import { keybinds, resetAllKeybinds, resetKeybind, setKeybind } from '../common/keybinds';
import { Theme, currentTheme, setTheme } from '../common/themes';
import Button from './button';
import Dropdown from './dropdown';
import FileInput from './file-input';
import Input from './input';
import { faRaspberryPi } from '@fortawesome/free-brands-svg-icons';
import {
  faBan,
  faCircleInfo,
  faCloudMoon,
  faGamepad,
  faKeyboard,
  faPalette,
  faPhotoFilm,
  faRefresh,
  faSun,
  faXmark
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component, useState } from 'react';

const GAMEPAD_SETTING_INPUTS: { setting: DrillGamepadSetting; tooltip: string }[] = [
  { setting: 'rackStick', tooltip: 'Maximum rack value from the left stick.' },
  { setting: 'drillStick', tooltip: 'Maximum drill value from the right stick.' },
  { setting: 'rackArrow', tooltip: 'Maximum rack value from the up/down arrows.' },
  { setting: 'drillArrow', tooltip: 'Maximum drill value from the left/right arrows.' }
];

function GamepadSettingInput({
  gamepadId,
  setting,
  tooltip
}: (typeof GAMEPAD_SETTING_INPUTS)[number] & {
  gamepadId: string;
}) {
  const [value, setValue] = useState(() => String(getDrillGamepadSettings(gamepadId)[setting]));

  const commit = () => {
    setValue(String(setDrillGamepadSetting(gamepadId, setting, value)));
  };

  return (
    <div className={styles['gamepad-setting']}>
      <Button className={styles['gamepad-setting-button']} tooltip={tooltip}>
        <FontAwesomeIcon icon={faCircleInfo} />
      </Button>
      <div className={styles['gamepad-setting-input']}>
        <input
          type='number'
          min='0'
          max={DEFAULT_DRILL_GAMEPAD_SETTINGS[setting]}
          step='1'
          value={value}
          aria-label={tooltip}
          onChange={(event) => setValue(event.currentTarget.value)}
          onBlur={commit}
          onKeyDown={(event) => {
            event.stopPropagation();
            if (event.key === 'Enter') event.currentTarget.blur();
            if (event.key === 'Escape') {
              setValue(String(getDrillGamepadSettings(gamepadId)[setting]));
              event.currentTarget.blur();
            }
          }}
        />
      </div>
      <Button
        className={styles['gamepad-setting-button']}
        tooltip='Reset value'
        onClick={() => setValue(String(resetDrillGamepadSetting(gamepadId, setting)))}
      >
        <FontAwesomeIcon icon={faRefresh} />
      </Button>
    </div>
  );
}

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

function readBackgroundImage(file: File) {
  return new Promise<BackgroundImage>((resolve, reject) => {
    const reader = new FileReader();

    reader.onload = () => {
      const dataUrl = reader.result;

      if (typeof dataUrl === 'string') {
        resolve({
          name: file.name,
          dataUrl
        });
      } else {
        reject(new Error(`Could not read ${file.name}.`));
      }
    };

    reader.onerror = () => reject(reader.error);
    reader.readAsDataURL(file);
  });
}

type State = {
  shown: boolean;
  listeningForNewKeybind: string | null;
  searchTerm: string;
  backgroundImagesLabel: string | null;
  connectedGamepadIds: string[];
};

export default class Settings extends Component<{}, State> {
  static defaultState: State = {
    shown: false,
    listeningForNewKeybind: null,
    searchTerm: '',
    backgroundImagesLabel: getBackgroundImagesLabel(),
    connectedGamepadIds: Array.from(gamepads.keys())
  };
  state = Settings.defaultState;

  componentDidMount() {
    window.addEventListener('gamepads-connect', this.gamepadsConnectHandler);
  }

  componentWillUnmount() {
    window.removeEventListener('gamepads-connect', this.gamepadsConnectHandler);
    window.removeEventListener('keydown', this.escHandler);
    window.removeEventListener('keydown', this.keyListener);
    window.removeEventListener('mousedown', this.clickToStopListeningForNewKeybindListener);
  }

  gamepadsConnectHandler = () => {
    const connectedGamepadIds = Array.from(gamepads.keys());
    const idsChanged =
      connectedGamepadIds.length !== this.state.connectedGamepadIds.length ||
      connectedGamepadIds.some((id, index) => id !== this.state.connectedGamepadIds[index]);

    if (idsChanged) this.setState({ connectedGamepadIds });
  };

  escHandler = (e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      this.hide();
    } else {
      // Focus the search input when any other key is pressed.
      // Saves the user a little bit of trouble when they forget to click the search box.
      const input = document.querySelector(`.${styles['search-input-parent']} input`);
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
      searchTerm: '',
      backgroundImagesLabel: getBackgroundImagesLabel(),
      connectedGamepadIds: Array.from(gamepads.keys())
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
    window.addEventListener('mousedown', this.clickToStopListeningForNewKeybindListener);
  }

  stopListeningForNewKeybind() {
    this.setState({ listeningForNewKeybind: null });
    window.removeEventListener('keydown', this.keyListener);
    window.addEventListener('keydown', this.escHandler);
    window.removeEventListener('mousedown', this.clickToStopListeningForNewKeybindListener);
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
          this.isSearchedFor(action + key + keyCodeToName(key) + 'keybinds') && (
            <div className={styles['keybind']} key={action}>
              <div className={styles['keybind-name']}>{action}</div>
              <div className={styles['keybind-controls']}>
                <Button
                  className={
                    styles['keybind-value'] +
                    (this.state.listeningForNewKeybind === action ? ` ${styles['keybind-value-listening']}` : '') +
                    (this.state.listeningForNewKeybind === null ? ` ${styles['keybind-value-hoverable']}` : '')
                  }
                  tooltip={this.state.listeningForNewKeybind === null ? 'Change this keybind.' : undefined}
                  onClick={() => {
                    if (this.state.listeningForNewKeybind) {
                      return;
                    }
                    this.startListeningForNewKeybind(action);
                  }}
                >
                  &nbsp;
                  {this.state.listeningForNewKeybind === action ? 'Waiting for key...' : keyCodeToName(key)}
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

    const showGamepads =
      this.state.connectedGamepadIds.length > 0 &&
      this.isSearchedFor(
        `gamepads controller max rack drill drilling stick arrow ${this.state.connectedGamepadIds
          .map(getGamepadName)
          .join(' ')}`
      );

    return (
      <div
        className={styles['settings-bg'] + (this.state.shown ? ` ${styles['shown']}` : '')}
        onClick={() => this.hide()}
      >
        <div className={styles['settings']} onClick={(e) => e.stopPropagation()}>
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
            <div className={styles['scrollable-options']} key={this.state.searchTerm}>
              {this.isSearchedFor('select color theme light mode dark mode berry purple') && (
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
                      defaultItemIndex={['dark', 'light', 'berry'].indexOf(currentTheme)}
                    />
                  </div>
                </>
              )}
              {(searchedKeybinds.length > 0 || this.isSearchedFor('reset all keybinds')) && (
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
              {showGamepads && (
                <>
                  <h2>
                    <FontAwesomeIcon icon={faGamepad} />
                    &nbsp;&nbsp;Gamepads
                  </h2>
                  {this.state.connectedGamepadIds.map((gamepadId) => (
                    <div className={styles['gamepad-settings']} key={gamepadId}>
                      <div className={styles['gamepad-settings-row']}>
                        <div className={styles['gamepad-name']} title={gamepadId}>
                          {getGamepadName(gamepadId)}
                        </div>
                        <div className={styles['gamepad-settings-inputs']}>
                          {GAMEPAD_SETTING_INPUTS.map((input) => (
                            <GamepadSettingInput gamepadId={gamepadId} key={input.setting} {...input} />
                          ))}
                        </div>
                      </div>
                    </div>
                  ))}
                </>
              )}
              {this.isSearchedFor('background image anime') && (
                <>
                  <h2>
                    <FontAwesomeIcon icon={faPhotoFilm} />
                    &nbsp;&nbsp;Background Image
                  </h2>
                  <div className={styles['background-selector']}>
                    <FileInput
                      accept='image/*'
                      multiple
                      emptyLabel={this.state.backgroundImagesLabel}
                      onChange={(files) => {
                        if (!files || files.length === 0) return;

                        Promise.all(Array.from(files).map(readBackgroundImage))
                          .then((images) => {
                            setBackgroundImages(images);
                            this.setState({ backgroundImagesLabel: getBackgroundImagesLabel(images) });
                            window.dispatchEvent(new Event('panel-manager-rerender'));
                          })
                          .catch(console.error);
                      }}
                      onClear={() => {
                        setBackgroundImages([]);
                        this.setState({ backgroundImagesLabel: null });
                        window.dispatchEvent(new Event('panel-manager-rerender'));
                      }}
                      canClearEmpty={!!this.state.backgroundImagesLabel}
                    />
                  </div>
                </>
              )}
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
