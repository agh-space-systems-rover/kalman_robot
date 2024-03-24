import styled from 'styled-components'

import { abortPositioning, startPositioning } from '../../api/requests'

const Wrapper = styled.div`
  grid-area: positioning;
`

export const ArmPositioning: () => JSX.Element = () => {
  //   const [actualPosition, setActualPosition] = useState({ jointId: '0', angle: '0' })

  const handleStart: (id: number) => void = (id: number) => {
    startPositioning(id)
  }

  const handleAbort: () => void = () => abortPositioning()

  const startButtons = [0, 1, 2, 3].map((v) => (
    <div key={v}>
      Joint {v + 1}:<button onClick={(): void => handleStart(v)}>start</button>
    </div>
  ))

  //   const handleSubmit: (event: FormEvent<HTMLFormElement>) => void = (event) => {
  //     event.preventDefault()
  //     setActualJointPosition(parseInt(actualPosition.jointId), parseFloat(actualPosition.angle))
  //   }

  //   const handleJointChange: (event: ChangeEvent<HTMLInputElement>) => void = (event) => {
  //     event.preventDefault()
  //     setActualPosition({ ...actualPosition, jointId: event.target.value })
  //   }

  //   const handleAngleChange: (event: ChangeEvent<HTMLInputElement>) => void = (event) => {
  //     event.preventDefault()
  //     setActualPosition({ ...actualPosition, angle: event.target.value })
  //   }

  return (
    <Wrapper>
      <h3>Arm Positioning</h3>
      <div>
        Start positioning:
        {startButtons}
        <button onClick={handleAbort}>abort</button>
      </div>
      {/* <div>
        Set actual position:
        <form onSubmit={handleSubmit}>
          <div>
            <label htmlFor='joint_id'>Joint ID</label>
            <input
              type='number'
              name='joint_id'
              min={0}
              max={5}
              value={actualPosition.jointId}
              onChange={handleJointChange}
            />
          </div>
          <div>
            <label htmlFor='position_radians'>Position: [rad]</label>
            <input
              type='number'
              step='0.01'
              name='position_radians'
              id='position_radians'
              value={actualPosition.angle}
              onChange={handleAngleChange}
            />
          </div>
          <button type='submit'>Submit</button>
        </form>
      </div> */}
    </Wrapper>
  )
}
