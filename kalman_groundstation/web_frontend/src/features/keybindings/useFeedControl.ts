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
  feed1: [
    'feed1.1Key',
    'feed1.2Key',
    'feed1.3Key',
    'feed1.4Key',
    'feed1.5Key',
    'feed1.6Key',
    'feed1.7Key',
    'feed1.8Key',
  ],
  feed2: [
    'feed2.1Key',
    'feed2.2Key',
    'feed2.3Key',
    'feed2.4Key',
    'feed2.5Key',
    'feed2.6Key',
    'feed2.7Key',
    'feed2.8Key',
  ],
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
