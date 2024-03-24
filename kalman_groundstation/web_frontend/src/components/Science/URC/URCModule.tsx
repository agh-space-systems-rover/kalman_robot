import './URCModule.css'
import './URCScience.css'

import React from 'react'

import { setScienceBacklight, setScienceHeater, setSciencePump, setScienceServo } from '../../../api/requests'
import type { Module } from '../../../store/Science/scienceTypes'

interface ModuleProps {
  module: Module
  moduleId: number
}

export const URCModule: React.FC<ModuleProps> = ({ module, moduleId }) => {
  const [isSealOpen, setIsSealOpen] = React.useState(false)
  const [isPumpOn, setIsPumpOn] = React.useState(false)
  const [isLightOn, setIsLightOn] = React.useState(false)
  const [isHeaterOn, setIsHeaterOn] = React.useState(false)

  const pump: (value: boolean) => void = (value) => {
    setIsPumpOn(value)
    setSciencePump(moduleId, value)
  }

  const heater: (value: boolean) => void = (value) => {
    setIsHeaterOn(value)
    setScienceHeater(moduleId, value)
  }

  const light: (value: boolean) => void = (value) => {
    setIsLightOn(value)
    setScienceBacklight(moduleId, value)
  }

  const seal: (value: number) => void = (value) => {
    setIsSealOpen(value > 0)
    setScienceServo(moduleId, value)
  }

  if (module.temperature > 85 && isHeaterOn) {
    heater(false)
  }

  // nie mamy zwrotek a chcemy mieć pewność ze jest wyłączone
  // pewnie można wywalić po URCu
  // useEffect(() => {
  //   if (!isHeaterOn) {
  //     const interval = setInterval(() => heater(false), 4000)
  //     return () => clearInterval(interval)
  //   }
  //   return () => {}
  // })

  return (
    <div className='container'>
      <h3>Module {moduleId} </h3>
      <hr style={{ backgroundColor: 'black', height: 1, width: '100%' }} />
      <div className={module.temperature > 80 ? 'flashing-background' : ''}>
        <div>Temperature: {module.temperature.toFixed(2)} </div>
      </div>
      <hr style={{ backgroundColor: 'black', height: 1 }} />
      <div className='science-buttons-grid'>
        <button className={`science-button ${isSealOpen ? 'last-pressed' : ''}`} onClick={(): void => seal(180)}>
          OPEN SEAL
        </button>
        <button className={`science-button ${!isSealOpen ? 'last-pressed' : ''}`} onClick={(): void => seal(0)}>
          CLOSE SEAL
        </button>
        <button className={`science-button ${isPumpOn ? 'last-pressed' : ''}`} onClick={(): void => pump(true)}>
          PUMP ON
        </button>
        <button className={`science-button ${!isPumpOn ? 'last-pressed' : ''}`} onClick={(): void => pump(false)}>
          PUMP OFF
        </button>
        <button className={`science-button ${isLightOn ? 'last-pressed' : ''}`} onClick={(): void => light(true)}>
          LIGHT ON
        </button>
        <button className={`science-button ${!isLightOn ? 'last-pressed' : ''}`} onClick={(): void => light(false)}>
          LIGHT OFF
        </button>
        <button className={`science-button ${isHeaterOn ? 'last-pressed' : ''}`} onClick={(): void => heater(true)}>
          HEATER ON
        </button>
        <button className={`science-button ${!isHeaterOn ? 'last-pressed' : ''}`} onClick={(): void => heater(false)}>
          HEATER OFF
        </button>
      </div>
    </div>
  )
}
