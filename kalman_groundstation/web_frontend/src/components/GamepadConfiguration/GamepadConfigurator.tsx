import { selectGamepad } from '../../store/Gamepads/gamepadSlice'
import { useAppSelector } from '../../store/storeHooks'
import { SingleGamepad } from './SingleGamepad'

const GamepadConfigurator: React.FC = () => {
  const gamepads = useAppSelector(selectGamepad)
  const autonomy = useAppSelector((state) => state.autonomy)

  const getColor: () => string = () =>
    autonomy.ueuos === 'unknown'
      ? 'rgba(255, 255, 255, 0.93)'
      : autonomy.ueuos === 'succeeded (G)'
      ? 'rgba(180, 255, 180, 0.93)'
      : autonomy.ueuos === 'manual (B)'
      ? 'rgba(180, 180, 255, 0.93)'
      : 'rgba(255, 180, 180, 0.93)'

  const list = gamepads.map((g) => {
    return <SingleGamepad key={g.index} gamepad={{ ...g }} />
  })

  return (
    <div style={{ gridArea: 'gamepads', overflow: 'scroll', backgroundColor: getColor() }}>
      <h3>Gamepads:</h3>
      {list}
    </div>
  )
}

export { GamepadConfigurator }
