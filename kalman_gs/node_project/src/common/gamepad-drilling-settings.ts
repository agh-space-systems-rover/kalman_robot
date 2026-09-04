const STORAGE_KEY_PREFIX = 'drill-gamepad-settings:';

type DrillGamepadSetting = 'rackStick' | 'drillStick' | 'rackArrow' | 'drillArrow';

type DrillGamepadSettings = Record<DrillGamepadSetting, number>;

const DEFAULT_DRILL_GAMEPAD_SETTINGS: DrillGamepadSettings = {
  rackStick: 20,
  drillStick: 100,
  rackArrow: 20,
  drillArrow: 100
};

const settingsCache = new Map<string, DrillGamepadSettings>();

const getStorageKey = (gamepadId: string) => `${STORAGE_KEY_PREFIX}${gamepadId}`;

const normalizeSetting = (setting: DrillGamepadSetting, value: unknown, fallback: number) => {
  const parsedValue = typeof value === 'number' ? value : Number(value);
  if (!Number.isFinite(parsedValue)) return fallback;

  return Math.max(0, Math.min(DEFAULT_DRILL_GAMEPAD_SETTINGS[setting], Math.round(parsedValue)));
};

const getDrillGamepadSettings = (gamepadId: string): DrillGamepadSettings => {
  const cachedSettings = settingsCache.get(gamepadId);
  if (cachedSettings) return cachedSettings;

  let storedSettings: Partial<DrillGamepadSettings> = {};
  const storedSettingsJson = localStorage.getItem(getStorageKey(gamepadId));

  if (storedSettingsJson) {
    try {
      const parsedSettings = JSON.parse(storedSettingsJson);
      if (typeof parsedSettings === 'object' && parsedSettings !== null && !Array.isArray(parsedSettings)) {
        storedSettings = parsedSettings;
      } else {
        localStorage.removeItem(getStorageKey(gamepadId));
      }
    } catch {
      localStorage.removeItem(getStorageKey(gamepadId));
    }
  }

  const settings = Object.fromEntries(
    (Object.keys(DEFAULT_DRILL_GAMEPAD_SETTINGS) as DrillGamepadSetting[]).map((setting) => [
      setting,
      normalizeSetting(setting, storedSettings[setting], DEFAULT_DRILL_GAMEPAD_SETTINGS[setting])
    ])
  ) as DrillGamepadSettings;

  settingsCache.set(gamepadId, settings);
  return settings;
};

const saveDrillGamepadSettings = (gamepadId: string, settings: DrillGamepadSettings) => {
  settingsCache.set(gamepadId, settings);
  localStorage.setItem(getStorageKey(gamepadId), JSON.stringify(settings));
};

const setDrillGamepadSetting = (gamepadId: string, setting: DrillGamepadSetting, rawValue: unknown) => {
  const currentSettings = getDrillGamepadSettings(gamepadId);
  const value = normalizeSetting(setting, rawValue, currentSettings[setting]);
  saveDrillGamepadSettings(gamepadId, { ...currentSettings, [setting]: value });
  return value;
};

const resetDrillGamepadSetting = (gamepadId: string, setting: DrillGamepadSetting) =>
  setDrillGamepadSetting(gamepadId, setting, DEFAULT_DRILL_GAMEPAD_SETTINGS[setting]);

export { DEFAULT_DRILL_GAMEPAD_SETTINGS, getDrillGamepadSettings, resetDrillGamepadSetting, setDrillGamepadSetting };
export type { DrillGamepadSetting, DrillGamepadSettings };
