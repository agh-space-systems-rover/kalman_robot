import { useEffect } from 'react'

import { translateScancode } from '../../store/Keybinds/keybindsSlice'
// import { useTranslateScancode } from '../../store/Keybinds/keybindsSlice'
import { addKey, removeKey } from '../../store/Keys/keysSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'

export const useKeybindings: () => void = () => {
  const dispatch = useAppDispatch()
  const selectedBinds = useAppSelector((state) => state.keybinds.binds)

  useEffect(() => {
    const useHandleKeyDown: (e: KeyboardEvent) => void = (e) => {
      const translatedKey = translateScancode(selectedBinds, e.code)
      if (e.ctrlKey) {
        if (e.code == 'KeyR') {
          e.preventDefault()
          e.stopPropagation()
        }
      }
      if (!e.repeat && document.activeElement?.tagName != 'INPUT') {
        dispatch(addKey(translatedKey))
      }
    }

    const useHandleKeyUp: (e: KeyboardEvent) => void = (e: KeyboardEvent) => {
      const translatedKey = translateScancode(selectedBinds, e.code)
      dispatch(removeKey(translatedKey))
    }

    document.addEventListener('keydown', useHandleKeyDown)
    document.addEventListener('keyup', useHandleKeyUp)
    return () => {
      document.removeEventListener('keydown', useHandleKeyDown)
      document.removeEventListener('keyup', useHandleKeyUp)
    }
  }, [])
}
