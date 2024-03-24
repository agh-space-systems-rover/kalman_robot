import type React from 'react'

import type { Feed } from '../../store/Feeds/feedsSlice'
import { selectFeeds } from '../../store/Feeds/feedsSlice'
import { useAppSelector } from '../../store/storeHooks'

interface SingleFeedProps {
  feed: Feed
  description: string
}

const SingleFeedIndicator: React.FC<SingleFeedProps> = ({ feed, description }: SingleFeedProps) => {
  const indicators = []
  for (let i = 1; i <= 8; i++) {
    const indicator = (
      <div key={i}>
        <span style={{ marginRight: 2, fontSize: 20 }}>{feed.camera == i ? '️☑️' : '🟥'}</span>
        {i == 4 ? ' ' : ''}
      </div>
    )
    indicators.push(indicator)
  }
  return (
    <div style={{ display: 'flex' }}>
      {indicators}
      <span style={{ marginLeft: 5 }}>{description}</span>
    </div>
  )
}
export const FeedIndicator: () => JSX.Element = () => {
  const feeds = useAppSelector(selectFeeds)

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
      <SingleFeedIndicator feed={feeds[0]} description='Whl' />
      <SingleFeedIndicator feed={feeds[1]} description='Arm' />
    </div>
  )
}
