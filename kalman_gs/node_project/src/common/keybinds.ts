const defaultKeybinds = {
  // Keyboard Driving
  'Drive Forward': null,
  'Drive Backward': null,
  'Turn Left': null,
  'Turn Right': null,
  'Rotate Left in Place': null,
  'Rotate Right in Place': null,
  'Hold to Modify Speeds and Turn Radius': null,
  // Feeds
  'Cycle Feed 1 Cameras Backwards': 'PageUp',
  'Cycle Feed 1 Cameras': 'PageDown',
  'Show Camera 1 on Feed 1': 'Digit1',
  'Show Camera 2 on Feed 1': 'Digit2',
  'Show Camera 3 on Feed 1': 'Digit3',
  'Show Camera 4 on Feed 1': 'Digit4',
  'Show Camera 5 on Feed 1': 'Digit5',
  'Show Camera 6 on Feed 1': 'Digit6',
  'Show Camera 7 on Feed 1': 'Digit7',
  'Show Camera 8 on Feed 1': 'Digit8',
  'Show Camera 1 on Feed 2': 'KeyQ',
  'Show Camera 2 on Feed 2': 'KeyW',
  'Show Camera 3 on Feed 2': 'KeyE',
  'Show Camera 4 on Feed 2': 'KeyR',
  'Show Camera 5 on Feed 2': 'KeyT',
  'Show Camera 6 on Feed 2': 'KeyY',
  'Show Camera 7 on Feed 2': 'KeyU',
  'Show Camera 8 on Feed 2': 'KeyI',
  'Hold to Change Cameras on Feed 2 not 1': null,
  // Arm
  'Set SpaceMouse Linear Scale to Slow (0.1)': 'KeyV',
  'Set SpaceMouse Linear Scale to Medium (0.3)': 'KeyB',
  'Set SpaceMouse Linear Scale to Fast (0.7)': 'KeyN',
  'Set SpaceMouse Rotational Scale to Slow (0.1)': 'KeyF',
  'Set SpaceMouse Rotational Scale to Medium (0.3)': 'KeyG',
  'Set SpaceMouse Rotational Scale to Fast (0.7)': 'KeyH'
};

// Copy default keybinds to keybinds.
let keybinds = Object.assign({}, defaultKeybinds);

// Load state from local storage if available.
const savedKeybinds = localStorage.getItem('keybinds');
if (savedKeybinds) {
  // keybinds = JSON.parse(savedKeybinds);
  // Object.assign in order to keep any new keybinds from defaultKeybinds.
  keybinds = Object.assign(keybinds, JSON.parse(savedKeybinds));
  // Remove keybinds that are not in defaultKeybinds.
  for (const key in keybinds) {
    if (!(key in defaultKeybinds)) {
      delete keybinds[key];
    }
  }
}

function getKeybind(action: string): string {
  if (keybinds[action] === undefined) {
    throw new Error(`Keybind for action "${action}" not found.`);
  }
  return keybinds[action];
}

function setKeybind(action: string, code: string) {
  keybinds[action] = code;
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

function resetKeybind(action: string): void {
  keybinds[action] = defaultKeybinds[action];
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

function resetAllKeybinds(): void {
  keybinds = Object.assign({}, defaultKeybinds);
  localStorage.setItem('keybinds', JSON.stringify(keybinds));
}

export { getKeybind, setKeybind, resetKeybind, resetAllKeybinds, keybinds };
