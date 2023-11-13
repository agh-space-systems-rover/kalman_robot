import { useEffect } from 'react'

import { addKey, removeKey } from '../../store/Keys/keysSlice'
import { useAppDispatch } from '../../store/storeHooks'

export const useKeybindings: () => void = () => {
  const dispatch = useAppDispatch()

  useEffect(() => {
    const handleKeyDown: (e: KeyboardEvent) => void = (e) => {
      if (e.ctrlKey) {
        if (e.code == 'KeyR') {
          e.preventDefault()
          e.stopPropagation()
        }
      }
      if (!e.repeat && document.activeElement?.tagName != 'INPUT') {
        dispatch(addKey(e.code))
      }
    }

    const handleKeyUp: (e: KeyboardEvent) => void = (e: KeyboardEvent) => {
      dispatch(removeKey(e.code))
    }

    document.addEventListener('keydown', handleKeyDown)
    document.addEventListener('keyup', handleKeyUp)
    return () => {
      document.removeEventListener('keydown', handleKeyDown)
      document.removeEventListener('keyup', handleKeyUp)
    }
  }, [])
}
