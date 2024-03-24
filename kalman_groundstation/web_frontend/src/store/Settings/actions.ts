import type { SettingsAction } from './constants'
import { SettingsActionType } from './constants'

export const changeUrl = (newUrl: string): SettingsAction => ({
  type: SettingsActionType.ChangeUrl,
  newUrl,
})

export const changeRefreshRate = (newRate: number): SettingsAction => ({
  type: SettingsActionType.ChangeRefreshRate,
  newRate,
})

export const disableAxisLocks = (value: boolean): SettingsAction => ({
  type: SettingsActionType.DisableAxisLocks,
  value,
})

export const disableGripper = (value: boolean): SettingsAction => ({
  type: SettingsActionType.DisableGripper,
  value,
})
