import { useEffect } from 'react'

import { setCamera } from '../../api/requests'
import { selectFeeds, setFeeds } from '../../store/Feeds/feedsSlice'
import { selectKeys } from '../../store/Keys/keysSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'

export interface KeyboardFeedControl {
  feed1: string[]
  feed2: string[]
}

const keyboardFeedControl: KeyboardFeedControl = {
  feed1: ['Digit1', 'Digit2', 'Digit3', 'Digit4', 'Digit5', 'Digit6', 'Digit7', 'Digit8'],
  feed2: ['KeyQ', 'KeyW', 'KeyE', 'KeyR', 'KeyT', 'KeyY', 'KeyU', 'KeyI'],
}

export const useKeyboardFeedControl: () => void = () => {
  const pressedKeys = useAppSelector(selectKeys)
  const feeds = useAppSelector(selectFeeds)
  const dispatch = useAppDispatch()

  useEffect(() => {
    for (const key of keyboardFeedControl.feed1) {
      if (pressedKeys.includes(key)) {
        dispatch(
          setFeeds(
            feeds.map((feed, idx) => {
              if (idx == 0) return { ...feed, camera: keyboardFeedControl.feed1.indexOf(key) + 1 }
              return feed
            }),
          ),
        )
        setCamera(1, feeds[0].camera)
      }
    }

    for (const key of keyboardFeedControl.feed2) {
      if (pressedKeys.includes(key)) {
        dispatch(
          setFeeds(
            feeds.map((feed, idx) => {
              if (idx == 1) return { ...feed, camera: keyboardFeedControl.feed2.indexOf(key) + 1 }
              return feed
            }),
          ),
        )
        setCamera(2, feeds[1].camera)
      }
    }
  }, [pressedKeys])
}
