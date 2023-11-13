import type { ChangeEvent, FormEvent } from 'react'
import { useEffect, useState } from 'react'
import styled from 'styled-components'

import { sendUsp } from '../../api/requests'
import { useAppSelector } from '../../store/storeHooks'
import type { UspParams } from '../AutonomyFront/Usp'

const Wrapper = styled.div`
  padding: 5px;

  input {
    border: 1px solid #000;
  }

  button {
    border: 1px solid #000;
    padding: 5px;
  }

  .preview {
    color: grey;
  }
`

interface Goal {
  latitude: number | null
  longitude: number | null
}

interface AzimuthGoal {
  distance: number // meters
  azimuth: number // degrees, clockwise from north
}

const initialAzimuthGoal: AzimuthGoal = {
  distance: 0.0,
  azimuth: 0.0,
}

const initialGoal: Goal = {
  latitude: null,
  longitude: null,
}

const defaultUspParams: UspParams = {
  autonomousDriving: true,
  goalSet: true,
  multipleWaypoints: false,
  frame: 4, // gps
  mode: 1, // simple goal
}

const stepDistance = 0.1
const minDistance = 0.0
const maxDistance = 1000.0
const stepAzimuth = 0.1
const minAzimuth = -360.0
const maxAzimuth = 360.0

export const AutonomyAzimuth: () => JSX.Element = () => {
  const gps = useAppSelector((state) => state.autonomy.gps)
  const [azimuthGoal, setAzimuthGoal] = useState<AzimuthGoal>(initialAzimuthGoal)
  const [goal, setGoal] = useState<Goal>(initialGoal)

  useEffect(() => {
    if (gps.position.latitude !== null && gps.position.longitude !== null) {
      setGoal(azimuthGoalToGoal(gps.position.latitude, gps.position.longitude, azimuthGoal))
    }
  }, [azimuthGoal])

  const handleSubmit: (event: FormEvent<HTMLFormElement>) => void = (event) => {
    event.preventDefault()
    if (goal.latitude !== null && goal.longitude !== null) {
      const uspParams: UspParams = {
        ...defaultUspParams,
        y: goal.longitude,
        x: goal.latitude, // TODO FIX ME REVERT X AND Y
      }

      sendUsp(uspParams)
    }
  }

  const handleChange: (event: ChangeEvent<HTMLInputElement>) => void = (event) => {
    event.preventDefault()
    const newAzimuthGoal = { ...azimuthGoal }
    if (event.target.name === 'distance') {
      newAzimuthGoal.distance = parseFloat(event.target.value)
    } else if (event.target.name === 'azimuth') {
      newAzimuthGoal.azimuth = parseFloat(event.target.value)
    }
    setAzimuthGoal(newAzimuthGoal)
  }

  const formatNumber: (value: number | null | undefined, precision: number) => string | number = (
    value: number | null | undefined,
    precision: number,
  ) => {
    if (value === null || value === undefined) {
      return 'unknown'
    } else {
      return value.toFixed(precision)
    }
  }

  const azimuthGoalToGoal: (lat: number, lon: number, azimuthGoal: AzimuthGoal) => Goal = (lat, lon, azimuthGoal) => {
    const R = 6378.1 * 1000

    const lat1 = lat * (Math.PI / 180)
    const lon1 = lon * (Math.PI / 180)
    const angularDistance = azimuthGoal.distance / R
    const trueBearing = azimuthGoal.azimuth * (Math.PI / 180)

    const lat2 = Math.asin(
      Math.sin(lat1) * Math.cos(angularDistance) + Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(trueBearing),
    )
    const lon2 =
      lon1 +
      Math.atan2(
        Math.sin(trueBearing) * Math.sin(angularDistance) * Math.cos(lat1),
        Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2),
      )

    const newLatitude = lat2 * (180 / Math.PI)
    const newLongitude = lon2 * (180 / Math.PI)

    return { latitude: newLatitude, longitude: newLongitude }
  }

  return (
    <Wrapper>
      <h5>Autonomy azimuth goals</h5>
      <form onSubmit={handleSubmit}>
        <label>Distance [m]: </label>
        <input
          type='number'
          name='distance'
          placeholder='Distance'
          step={stepDistance}
          min={minDistance}
          max={maxDistance}
          defaultValue='0'
          onChange={(event: ChangeEvent<HTMLInputElement>): void => handleChange(event)}
        />
        <br />
        <label>Azimuth [degs clockwise from N]: </label>
        <input
          type='number'
          name='azimuth'
          placeholder='Azimuth'
          step={stepAzimuth}
          min={minAzimuth}
          max={maxAzimuth}
          defaultValue='0'
          onChange={(event: ChangeEvent<HTMLInputElement>): void => handleChange(event)}
        />
        <br />
        <div className='preview'>
          <label>Goal latitude: </label>
          {formatNumber(goal.latitude, 7)}
          <br />
          <label>Goal longitude: </label>
          {formatNumber(goal.longitude, 7)}
        </div>
        <br />
        <button type='submit'>Send goal</button>
      </form>
    </Wrapper>
  )
}
