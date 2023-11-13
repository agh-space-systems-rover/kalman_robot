import type React from 'react'
import { useState } from 'react'
import styled from 'styled-components'

import { changeRefreshRate, changeUrl, disableAxisLocks, disableGripper } from '../../store/Settings/actions'
import { useAppDispatch, useAppSelector } from '../../store/storeHooks'

const Wrapper = styled.div`
  grid-area: settings;
  padding: 4px;
  display: flex;
  flex-direction: column;
  gap: 8px;
  text-align: center;

  & > h5 {
    font-size: 24px;
  }
`

export const Settings: () => JSX.Element = () => {
  const backUrlFromStore = useAppSelector((state) => state.settings.backUrl)
  const refreshRateFromStore = useAppSelector((state) => state.settings.refreshRate)
  const axisLocks = useAppSelector((state) => state.settings.disableAxisLocks)
  const gripper = useAppSelector((state) => state.settings.disableGripper)
  const [refresh, changeRefresh] = useState(refreshRateFromStore)
  const [backUrl, changeBackUrl] = useState(backUrlFromStore)
  const dispatch = useAppDispatch()

  return (
    <Wrapper>
      <h5>Settings</h5>
      <div>
        <label htmlFor='refreshRate'>Gamepads refresh rate</label>
        <input
          type='number'
          id='refreshRate'
          name='refreshRate'
          value={refresh}
          onWheel={(e: React.WheelEvent<HTMLInputElement>): void => e.currentTarget.blur()}
          onChange={(e: React.ChangeEvent<HTMLInputElement>): void => changeRefresh(parseInt(e.target.value))}
        />
        <button onClick={(): void => dispatch(changeRefreshRate(refresh))}>Set refresh rate</button>
      </div>
      <div>
        <input
          type='text'
          value={backUrl}
          onChange={(e: React.ChangeEvent<HTMLInputElement>): void => changeBackUrl(e.target.value)}
        />
        <button onClick={(): void => dispatch(changeUrl(backUrl))}>Set new url</button>
      </div>
      <div>
        <input
          type='checkbox'
          name='disableAxisLocks'
          checked={axisLocks}
          id={'cbAxis'}
          onChange={(): void => dispatch(disableAxisLocks(!axisLocks))}
        />
        <label htmlFor='cbAxis'>Disable axis locks</label>
      </div>
      <div>
        <input
          type='checkbox'
          name='disableGripper'
          checked={gripper}
          id={'cbGripper'}
          onChange={(): void => dispatch(disableGripper(!gripper))}
        />
        <label htmlFor='cbGripper'>Disable gripper spinning</label>
      </div>
    </Wrapper>
  )
}
