import type { Reducer } from 'redux'

import * as C from './constants'

export const settingsReducer: Reducer<C.SettingsState, C.SettingsAction> = (
  state: C.SettingsState = C.initialSettingsState,
  action: C.SettingsAction,
): C.SettingsState => {
  switch (action.type) {
    case C.SettingsActionType.ChangeUrl:
      return {
        ...state,
        backUrl: action.newUrl,
      }
    case C.SettingsActionType.ChangeRefreshRate:
      return {
        ...state,
        refreshRate: action.newRate,
      }
    case C.SettingsActionType.DisableGripper:
      return {
        ...state,
        disableGripper: action.value,
      }
    case C.SettingsActionType.DisableAxisLocks:
      return {
        ...state,
        disableAxisLocks: action.value,
      }
    default:
      return state
  }
}
