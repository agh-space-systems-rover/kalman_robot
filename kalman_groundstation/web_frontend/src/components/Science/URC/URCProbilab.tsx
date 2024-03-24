import './URCProbilab.css'
import './URCScience.css'

import React from 'react'

import { setScienceAux } from '../../../api/requests'

const PROBILAB_MODULE_ID = 2

export const URCProbilab: () => JSX.Element = () => {
  const [isProbilabOpen, setIsProbilabOpen] = React.useState(false)

  const probilab: (value: boolean) => void = (value) => {
    setIsProbilabOpen(value)
    setScienceAux(PROBILAB_MODULE_ID, value)
  }

  return (
    <div className='probilab'>
      <button className={`science-button ${isProbilabOpen ? 'last-pressed' : ''}`} onClick={(): void => probilab(true)}>
        PROBILAB ON
      </button>
      <button
        className={`science-button ${!isProbilabOpen ? 'last-pressed' : ''}`}
        onClick={(): void => probilab(false)}
      >
        PROBILAB OFF
      </button>
    </div>
  )
}
