import './Temperatures.css'

import { useAppSelector } from '../../store/storeHooks'

interface CellProps {
  data: {
    motor: number
    turn: number
    overheating: boolean
  }
}

export const Cell: (data: CellProps) => JSX.Element = ({ data }: CellProps) => {
  return (
    <div
      style={{
        width: '30%',
        border: '1px solid black',
        margin: '10%',
      }}
      className={data.overheating ? 'flashing-background' : ''}
    >
      <div>{data.turn}</div>
      <div>{data.motor}</div>
    </div>
  )
}

export const Temperatures: () => JSX.Element = () => {
  const temps = useAppSelector((state) => state.motors.temperature)
  if (!temps) return <div>Unknown motors temperatures</div>
  return (
    <div>
      <h3>Motor temperatures</h3>
      <div style={{ display: 'flex' }}>
        <Cell data={temps.fl} />
        <Cell data={temps.fr} />
      </div>
      <div style={{ display: 'flex' }}>
        <Cell data={temps.bl} />
        <Cell data={temps.br} />
      </div>
    </div>
  )
}
