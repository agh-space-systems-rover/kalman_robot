const applyDeadzone: (axes: readonly number[], deadzone?: number) => number[] = (
  axes: readonly number[],
  deadzone = 0.1,
) => {
  if (deadzone > 1 || deadzone < 0) {
    console.error('Incorrect dead zone value for thumb stick. Expected: float from range (0,1)')
    return axes.slice()
  }
  const newAxes = axes.map((axis) => {
    if (Math.abs(axis) < deadzone) return 0
    return ((Math.abs(axis) - deadzone) / (1 - deadzone)) * Math.sign(axis)
  })
  return newAxes
}

export { applyDeadzone }
