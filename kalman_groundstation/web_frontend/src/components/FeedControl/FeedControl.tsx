import type { ChangeEvent } from 'react'
import styled from 'styled-components'

import { setCamera, setChannel, setPower } from '../../api/requests'
import { selectFeeds, setFeeds } from '../../store/Feeds/feedsSlice'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'

const Wrapper = styled.div`
  grid-area: feed;
  overflow: hidden;
`

const SingleFeedWrapper = styled.div`
  width: 50%;
  & > form > * {
    flex: 1;
  
  input {
    width: 50px;
  }
`
interface SingleFeedProps {
  feedId: number
}

const SingleFeed: (props: SingleFeedProps) => JSX.Element = ({ feedId }: SingleFeedProps) => {
  const feeds = useAppSelector(selectFeeds)
  const dispatch = useAppDispatch()

  const handleCameraSubmit: () => void = () => {
    setCamera(feedId + 1, feeds[feedId].camera)
  }

  const handleChannelSubmit: (ass: boolean) => void = (ass) => {
    setChannel(ass, feedId + 1, feeds[feedId].channel)
  }

  const handlePowerSubmit: () => void = () => {
    setPower(feedId + 1, feeds[feedId].power)
  }

  return (
    <SingleFeedWrapper>
      <h3>Feed {feedId}</h3>
      <div style={{ margin: '10px' }}>
        <label>Camera</label>
        <div>
          <input
            type='number'
            value={feeds[feedId].camera}
            min={1}
            max={8}
            onChange={(e: ChangeEvent<HTMLInputElement>): void =>
              dispatch(
                setFeeds(
                  feeds.map((feed, idx) => {
                    if (idx == feedId) return { ...feed, camera: parseInt(e.target.value) }
                    return feed
                  }),
                ),
              )
            }
          />
          <button onClick={handleCameraSubmit}>Send</button>
        </div>
      </div>

      <div style={{ margin: '10px', display: 'block' }}>
        <label>Channel</label> <br />
        <div>
          <input
            type='number'
            value={feeds[feedId].channel}
            min={1}
            max={40}
            onChange={(e: ChangeEvent<HTMLInputElement>): void =>
              dispatch(
                setFeeds(
                  feeds.map((feed, idx) => {
                    if (idx == feedId) return { ...feed, channel: parseInt(e.target.value) }
                    return feed
                  }),
                ),
              )
            }
          />
          <div>
            <button style={{ display: 'block' }} onClick={(): void => handleChannelSubmit(false)}>
              Send
            </button>
            <button style={{ display: 'block' }} onClick={(): void => handleChannelSubmit(true)}>
              Dupa
            </button>
          </div>
        </div>
      </div>

      <div style={{ margin: '10px', display: 'block' }}>
        <label>Power</label> <br />
        <div>
          <input
            type='number'
            value={feeds[feedId].power}
            min={1}
            max={4}
            onChange={(e: ChangeEvent<HTMLInputElement>): void =>
              dispatch(
                setFeeds(
                  feeds.map((feed, idx) => {
                    if (idx == feedId) return { ...feed, power: parseInt(e.target.value) }
                    return feed
                  }),
                ),
              )
            }
          />
          <button onClick={handlePowerSubmit}>Send</button>
        </div>
      </div>
    </SingleFeedWrapper>
  )
}

export const FeedControl: () => JSX.Element = () => {
  const autonomy = useAppSelector((state) => state.autonomy)

  const getColor: () => string = () =>
    autonomy.ueuos === 'unknown'
      ? 'rgba(255, 255, 255, 0.93)'
      : autonomy.ueuos === 'succeeded (G)'
      ? 'rgba(180, 255, 180, 0.93)'
      : autonomy.ueuos === 'manual (B)'
      ? 'rgba(180, 180, 255, 0.93)'
      : 'rgba(255, 180, 180, 0.93)'

  return (
    <Wrapper style={{ backgroundColor: getColor() }}>
      <h2>Camera Feed</h2>
      <div style={{ display: 'flex' }}>
        <SingleFeed feedId={0}></SingleFeed>
        <SingleFeed feedId={1}></SingleFeed>
      </div>
    </Wrapper>
  )
}
