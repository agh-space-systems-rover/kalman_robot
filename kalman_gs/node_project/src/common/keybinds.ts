const defaultKeybinds = {
    "Drive Forward": "KeyW",
    "Drive Backward": "KeyS",
    "Turn Left": "KeyA",
    "Turn Right": "KeyD",
    "Rotate Left in Place": "KeyQ",
    "Rotate Right in Place": "KeyE",
    "Increase Speed": "ArrowUp",
    "Decrease Speed": "ArrowDown",
    "Decrease Turn Radius": "ArrowRight",
    "Increase Turn Radius": "ArrowLeft",
    "Cycle Feed 1 Cameras Backwards": "Home",
    "Cycle Feed 1 Cameras": "End",
    "Cycle Feed 2 Cameras Backwards": "PageUp",
    "Cycle Feed 2 Cameras": "PageDown",
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
    if (!keybinds[action]) {
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
