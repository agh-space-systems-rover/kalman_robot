import type { ChangeEvent, FormEvent } from 'react'
import { useState } from 'react'
import styled from 'styled-components'

interface Goal {
  latitude: string
  longitude: string
}

const Wrapper = styled.div`
  grid-area: goals;
  padding: 5px;
`

export const AutonomyGoals: () => JSX.Element = () => {
  const [goals, setGoals] = useState<Goal[]>([{ latitude: '0', longitude: '0' }])

  const handleSubmit: (event: FormEvent<HTMLFormElement>) => void = (event) => {
    event.preventDefault()
    console.log('Send goals')
    // dodać konwersję na liczby ze stringów
    console.log(goals)
  }

  const handleChange: (event: ChangeEvent<HTMLInputElement>, index: number) => void = (event, index) => {
    event.preventDefault()
    const newGoals = [...goals]
    if (event.target.name === 'latitude') {
      newGoals[index].latitude = event.target.value
    }
    if (event.target.name === 'longitude') {
      newGoals[index].longitude = event.target.value
    }
    setGoals(newGoals)
  }

  const handleRemove: (index: number) => void = (index: number) => {
    setGoals(goals.filter((v, i) => i !== index))
  }

  const handleNew: () => void = () => {
    setGoals([...goals, { latitude: '0', longitude: '0' }])
  }

  const inputs = goals.map((goal, index) => (
    <div key={index}>
      <label htmlFor='latitude'>Latitude:</label>
      <input
        type='number'
        name='latitude'
        value={goal.latitude}
        onChange={(event: ChangeEvent<HTMLInputElement>): void => handleChange(event, index)}
      />
      <label htmlFor=''>Longitude:</label>
      <input
        type='number'
        name='longitude'
        value={goal.longitude}
        onChange={(event: ChangeEvent<HTMLInputElement>): void => handleChange(event, index)}
      />
      <button type='button' hidden={goals.length < 2} onClick={(): void => handleRemove(index)}>
        remove
      </button>
    </div>
  ))

  return (
    <Wrapper>
      <h3>Sending goals</h3>
      <form onSubmit={handleSubmit}>
        {inputs}
        <button type='button' onClick={handleNew}>
          Add goal
        </button>
        <br />
        <button type='submit'>Send goals</button>
      </form>
      <div style={{ fontSize: '5px' }}>keidyś będzie ładnie &trade;</div>
    </Wrapper>
  )
}
