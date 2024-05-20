import { createSlice } from '@reduxjs/toolkit'

import type { GamepadValues } from '../../features/gamepads/gamepadTypes'
import type { gamepadFeatureButton, gamepadSettings, keyMapping, mappingParameters } from './keybindsTypes'
import { initialUserProfile } from './keybindsTypes'

const keybindsSlice = createSlice({
  name: 'userProfile',
  initialState: initialUserProfile,
  reducers: {},
})

// TODO zapytać o zmiane nazwy na userProfile i rozbicie na gameBindSlice itp (ZMIENIC NAZWY ROZJEBVAC NA PLIKI)

// TODO use typescript map

export const translateScancode = (selectedBindPair: keyMapping[], scancode: string): string => {
  let boundTarget
  for (const bindPair of selectedBindPair) if (bindPair.key == scancode) boundTarget = bindPair.target
  if (boundTarget == undefined) return 'keyUnbound'
  return boundTarget
}

export const translateGamepadButton = (selectedBindPair: gamepadFeatureButton[], target: string): number => {
  let boundScancode
  for (const bindPair of selectedBindPair) if (bindPair.target == target) boundScancode = bindPair.button
  if (boundScancode == undefined) return -1
  return boundScancode
}

// arduino's map()
export const mapLinear = (value: number, parameters: mappingParameters): number => {
  const inR = parameters.inputRange
  const outR = parameters.outputRange
  const calculatedValue = outR.low + ((outR.high - outR.low) * (value - inR.low)) / (inR.high - inR.low)
  if (!parameters.clipping) return calculatedValue
  if (calculatedValue > outR.high) return outR.high
  if (calculatedValue < outR.low) return outR.low

  return calculatedValue
}

export const translateGamepadAxis = (
  currentValue: GamepadValues,
  selectedGamepadBinds: gamepadSettings,
  target: string,
): number => {
  const selectedBindPair = selectedGamepadBinds.axisFeatures
  const selectedVirtualBindPair = selectedGamepadBinds.virtualAxes
  // below checking standard axes, todo add map() below
  // for (const bindPair of selectedBindPair) if (bindPair.target == target) return currentValue.axes[bindPair.axis]
  for (const bindPair of selectedBindPair)
    if (bindPair.target == target) {
      return mapLinear(currentValue.axes[bindPair.axis], bindPair.valueMapping)
    }

  // check virtual if there is no physical axes
  for (const bindPair of selectedVirtualBindPair)
    if (bindPair.target == target) {
      let sum = 0
      for (const currentAxe of bindPair.sourceAxes) {
        sum += translateGamepadAxis(currentValue, selectedGamepadBinds, currentAxe)
      }
      return sum
    }
  return 0 // if no axis found, todo check to null/undefined
}

export const keybindsReducer = keybindsSlice.reducer
