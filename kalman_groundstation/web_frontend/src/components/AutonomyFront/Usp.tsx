import React, { useState } from 'react'
import styled from 'styled-components'

import {
  clearCostmap,
  sendAutonomy,
  sendUeuosState,
  sendUsp,
  sendUspRequest,
  sendWaypointsGet,
  sendWaypointsSet,
  setMaxVelocityAtRover,
} from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'

const UspWrapper = styled.div`
  grid-area: dynamic;
  overflow: scroll;
  padding: 4px;
  & > h5 {
    font-size: 24px;
  }
`

const enum AutonomyHint {
  Unknown,
  On,
  Off,
}

export interface UspParams {
  x?: number
  y?: number
  autonomousDriving?: boolean
  goalSet?: boolean
  multipleWaypoints?: boolean
  frame?: number
  mode?: number
  tag1?: number
  tag2?: number
  args?: string
}

const initialParams: UspParams = {
  x: 4,
  y: 2,
  autonomousDriving: false,
  goalSet: false,
  multipleWaypoints: false,
  frame: 0,
  mode: 0,
  tag1: 1,
  tag2: 2,
  args: '',
}

export const Usp: () => JSX.Element = () => {
  const [params, setParams] = React.useState<UspParams>(initialParams)
  const [autonomyHint, setAutonomyHint] = useState<AutonomyHint>(AutonomyHint.Unknown)
  const [waypoints, setWaypoints] = React.useState<[number, number][]>([])
  const [maxVelocity, setMaxVeloctiy] = React.useState<number>(50)
  const autonomy = useAppSelector((state) => state.autonomy)

  const submit: () => void = () => {
    console.log(params)
    sendUsp(params)
  }

  const submitAutonomyOn: () => void = () => {
    setAutonomyHint(AutonomyHint.On)
    sendAutonomy(true)
  }
  const submitAutonomyOff: () => void = () => {
    setAutonomyHint(AutonomyHint.Off)
    sendAutonomy(false)
  }

  const flatWaypointsToTuples = (waypointsFlat: number[]): [number, number][] => {
    const waypoints: [number, number][] = []
    for (let i = 0; i < waypointsFlat.length; i = i + 2) {
      waypoints.push([waypointsFlat[i], waypointsFlat[i + 1]])
    }
    return waypoints
  }

  const styleWaypoints: (waypoints: [number, number][]) => JSX.Element[] = (waypoints: [number, number][]) => {
    return waypoints.map((item, index) => {
      return (
        <div key={index}>
          x:{item[0]}, y:{item[1]}
        </div>
      )
    })
  }

  const blurOnWheel: (e: React.WheelEvent<HTMLInputElement | HTMLSelectElement>) => void = (e) => {
    e.currentTarget.blur()
  }

  return (
    <UspWrapper>
      <h5>United Supervisor Protocol</h5>
      <b>Autonomous driving: </b>
      <input
        type='checkbox'
        min={0}
        max={1}
        checked={params.autonomousDriving}
        onChange={(): void => setParams((s) => ({ ...s, autonomousDriving: !s.autonomousDriving }))}
      />
      {autonomy.usp?.autonomousDriving ? 'True' : 'False'}
      <br />
      <b>Autonomous driving mode: </b>
      <select
        value={params.mode}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLSelectElement>): void =>
          setParams((s) => ({ ...s, mode: parseInt(e.target.value) }))
        }
      >
        <option value='0'>None(0)</option>
        <option value='1'>Simple goal(1)</option>
        <option value='2'>Single tag goal(2)</option>
        <option value='3'>Gate goal(3)</option>
      </select>
      {autonomy.usp?.mode}
      <br />
      <b>X: </b>
      <input
        type='number'
        // min={-90}
        // max={90}
        // step={0.000001}
        value={params.x}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void =>
          setParams((s) => ({ ...s, x: parseFloat(e.target.value) }))
        }
      />
      {autonomy.usp?.x}
      <button
        onClick={(): void => {
          const x: number = params.x === undefined ? 0.0 : params.x
          const y: number = params.y === undefined ? 0.0 : params.y
          setWaypoints(waypoints.concat([[x, -y]]))
          setParams({ ...params, multipleWaypoints: true })
        }}
        style={{
          border: 'solid 2px "#ccc"}',
          padding: '2px',
        }}
      >
        Add local goal to waypoint
      </button>
      <br />
      <b>Y: </b>
      <input
        type='number'
        // min={-90}
        // max={90}
        // step={0.000001}
        value={params.y}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void =>
          setParams((s) => ({ ...s, y: parseFloat(e.target.value) }))
        }
      />{' '}
      {autonomy.usp?.y}
      <button
        onClick={(): void => {
          setWaypoints([])
          setParams({ ...params, multipleWaypoints: false })
        }}
        style={{
          border: 'solid 2px "#ccc"}',
          padding: '2px',
        }}
      >
        Remove waypoints
      </button>
      <br />
      <b>Multiple Waypoints: </b>
      <input
        type='checkbox'
        min={0}
        max={1}
        checked={params.multipleWaypoints}
        onChange={(): void => setParams((s) => ({ ...s, multipleWaypoints: !s.multipleWaypoints }))}
      />{' '}
      <div style={{ display: 'flex' }}>
        <div style={{ display: 'flex', flexDirection: 'column', textAlign: 'center' }}>
          <h2>~~ Requested waypoints ~~</h2>
          {styleWaypoints(waypoints)}
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', textAlign: 'center' }}>
          <h2>~~ Usp waypoints ~~</h2>
          {styleWaypoints(
            autonomy.waypoints === undefined || autonomy.waypoints === null
              ? []
              : flatWaypointsToTuples(autonomy.waypoints),
          )}
        </div>
      </div>
      <button onClick={(): void => sendWaypointsSet(waypoints)}>submit waypoints</button>{' '}
      <button onClick={(): void => sendWaypointsGet()}>get waypoints from DR</button>
      <br />
      <br />
      <b>Goal Set: </b>
      <input
        type='checkbox'
        min={0}
        max={1}
        checked={params.goalSet}
        onChange={(): void => setParams((s) => ({ ...s, goalSet: !s.goalSet }))}
      />{' '}
      {autonomy.usp?.goalSet ? 'True' : 'False'}
      <br />
      <button onClick={(): void => submit()}>submit GOAL</button>
      <br />
      <button onClick={(): void => sendUspRequest()}>GET USP</button>
      <br />
      <button
        onClick={(): void => submitAutonomyOn()}
        style={{
          border: `solid 2px ${autonomyHint === AutonomyHint.On ? '#ef5350' : '#ccc'}`,
          padding: '2px',
        }}
      >
        autonomy ON
      </button>
      <br />
      <button
        onClick={(): void => submitAutonomyOff()}
        style={{
          border: `solid 2px ${autonomyHint === AutonomyHint.Off ? '#ef5350' : '#ccc'}`,
          padding: '2px',
        }}
      >
        autonomy OFF
      </button>
      <br />
      <button onClick={(): void => clearCostmap()}>Clear costmap!</button>
      <br />
      Tag 1 ID:{' '}
      <input
        type='number'
        value={params.tag1}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void =>
          setParams((s) => ({ ...s, tag1: parseInt(e.target.value) }))
        }
      />
      <br />
      Tag 2 ID:{' '}
      <input
        type='number'
        value={params.tag2}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void =>
          setParams((s) => ({ ...s, tag2: parseInt(e.target.value) }))
        }
      />
      <br />
      Frame:{' '}
      <select
        value={params.frame}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLSelectElement>): void =>
          setParams((s) => ({ ...s, frame: parseInt(e.target.value) }))
        }
      >
        <option value='0'>None(0)</option>
        <option value='1'>base_link(1)</option>
        <option value='2'>map(2)</option>
        <option value='3'>odom(3)</option>
        <option value='4'>gps(4)</option>
      </select>
      <br />
      Args:{' '}
      <input
        type='text'
        value={params.args}
        onWheel={blurOnWheel}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void => setParams((s) => ({ ...s, args: e.target.value }))}
      />
      <br />
      Autonomy max velocity:{' '}
      <input
        type='number'
        value={maxVelocity}
        onChange={(e: React.ChangeEvent<HTMLInputElement>): void => setMaxVeloctiy(parseInt(e.target.value))}
      />
      <br />
      <button onClick={(): void => setMaxVelocityAtRover(maxVelocity)}>Send max velocity!</button>
      <br />
      UEUOS:
      <br />
      <div>
        <button onClick={(): void => sendUeuosState(1)} style={{ margin: 3 }}>
          RED
        </button>
        <button onClick={(): void => sendUeuosState(2)} style={{ margin: 3 }}>
          BLUE
        </button>
        <button onClick={(): void => sendUeuosState(3)} style={{ margin: 3 }}>
          GREEN
        </button>
      </div>
    </UspWrapper>
  )
}
