export interface SettingsState {
  backUrl: string
  disableAxisLocks: boolean
  disableGripper: boolean
  refreshRate: number // updates per second
}

export enum SettingsActionType {
  ChangeUrl = 'Settings/ChangeUrl',
  ChangeRefreshRate = 'Settings/ChangeRefreshRate',
  DisableAxisLocks = 'Settings/DisableAxisLocks',
  DisableGripper = 'Settings/DisableGripper',
}

export type SettingsAction =
  | {
      type: SettingsActionType.ChangeUrl
      newUrl: string
    }
  | {
      type: SettingsActionType.ChangeRefreshRate
      newRate: number
    }
  | {
      type: SettingsActionType.DisableAxisLocks
      value: boolean
    }
  | {
      type: SettingsActionType.DisableGripper
      value: boolean
    }

export const initialSettingsState: SettingsState = {
  backUrl: 'localhost:8000',
  disableAxisLocks: true,
  disableGripper: false,
  refreshRate: 10, // updates per second
}
