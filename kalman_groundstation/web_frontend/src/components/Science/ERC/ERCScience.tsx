import * as _ from 'lodash'
import type { ChangeEvent } from 'react'
import { useCallback, useEffect, useState } from 'react'
import styled from 'styled-components'

import { openCloseSampleERC, requestWeightERC, setCarousel, tareWeight } from '../../../api/requests'
import { useAppSelector } from '../../../store/storeHooks'
import { Panorama } from '../Panorama'
import { SmartProbe } from '../SmartProbe'

const ColumnWrapper = styled.div`
  padding: 4px;
  display: flex;
  flex-direction: column;
  gap: 8px;
  text-align: center;
  & > h5 {
    font-size: 24px;
  }
  border: 1px solid black;
  & > button {
    border: 1px solid black;
  }
  & > button:hover {
    background-color: #ccc;
  }
  & > button:active {
    background-color: #aaa;
  }
`
const samplePositions = [5, 27, 52, 77]

export const ERCScience: () => JSX.Element = () => {
  const [carouselValue, setCarouselValue] = useState<number>(50)
  const weight = useAppSelector((state) => state.science.ERCWeight)

  const throttleSetter = useCallback(
    _.throttle((value: number) => {
      setCarousel(value)
    }, 200),
    [],
  )

  useEffect(() => {
    throttleSetter(carouselValue)
  }, [carouselValue])

  const sampleButtons = [0, 1, 2, 3].map((id) => (
    <button
      style={{ margin: 2, padding: '1% 10% 1% 10%' }}
      key={id}
      onClick={(): void => setCarouselValue(samplePositions[id])}
    >
      Sample {id}
    </button>
  ))

  return (
    <div>
      <h1>Science</h1>
      <div style={{ display: 'flex', flex: 1, justifyContent: 'space-around' }}>
        <ColumnWrapper>
          {sampleButtons}
          <div style={{ display: 'flex' }}>
            <span style={{ marginRight: 10 }}>{carouselValue}</span>

            <input
              type='range'
              min='0'
              max='100'
              step='1'
              value={carouselValue}
              onChange={(e: ChangeEvent<HTMLInputElement>): void => {
                setCarouselValue(parseInt(e.target.value))
              }}
            />
          </div>
          <div>
            <button onClick={(): void => openCloseSampleERC(true)} style={{ margin: 2, padding: 4 }}>
              Open
            </button>
            <button onClick={(): void => openCloseSampleERC(false)} style={{ margin: 2, padding: 4 }}>
              Close
            </button>
            <button onClick={requestWeightERC} style={{ margin: 2, padding: 4 }}>
              Request weight
            </button>
            <button onClick={tareWeight} style={{ margin: 2, padding: 4 }}>
              Tare weight
            </button>
          </div>
          <span style={{ fontSize: 25 }}>Weight: {weight}g</span>
        </ColumnWrapper>
        <SmartProbe />
        <Panorama />
      </div>
    </div>
  )
}
