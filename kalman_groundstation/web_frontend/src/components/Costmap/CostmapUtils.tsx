import { cellColors } from './Costmap'

export const Row: (values: number[], rowId: number) => JSX.Element = (values: number[], rowId: number) => {
  return (
    <div style={{ display: 'flex', height: '100%', alignItems: 'stretch' }}>
      {values.map((cellState, columnId) => Cell(cellState, rowId, columnId))}
    </div>
  )
}

export const Cell: (state: number, rowId: number, columnId: number) => JSX.Element = (
  state: number,
  rowId: number,
  columnId: number,
) => {
  const style: React.CSSProperties = {
    flex: 1,
    margin: '1px',
  }

  let color = cellColors.UNKNOWN
  switch (state) {
    case 1:
      color = cellColors.EMPTY
      break
    case 2:
      color = cellColors.OBSTACLE
      break
  }
  style.backgroundColor = color
  const normalBorder = `5px solid ${color}`

  const border = '5px solid black'
  style.borderTop = normalBorder
  style.borderBottom = normalBorder
  style.borderLeft = normalBorder
  style.borderRight = normalBorder
  if (columnId == 9) {
    if (rowId == 9) {
      style.borderTop = border
      style.borderLeft = border
    }
    if (rowId == 10) {
      style.borderBottom = border
      style.borderLeft = border
    }
  }
  if (columnId == 10) {
    if (rowId == 9) {
      style.borderTop = border
      style.borderRight = border
    }
    if (rowId == 10) {
      style.borderBottom = border
      style.borderRight = border
    }
  }
  return <div style={style} />
}

export const infoDiv: (color: string, label: string) => JSX.Element = (color: string, label: string) => (
  <div style={{ display: 'flex', alignItems: 'center', marginRight: '10px' }}>
    <div
      style={{
        width: '20px',
        borderRadius: '5px',
        height: '17px',
        backgroundColor: color,
      }}
    />
    <div style={{ padding: '2px' }}>{label}</div>
  </div>
)
