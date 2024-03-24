import './HamburgerButton.css'

import type { InputHTMLAttributes } from 'react'
import type React from 'react'

const HamburgerButton: React.FC<InputHTMLAttributes<HTMLInputElement>> = ({ onClick, checked }) => {
  return (
    <>
      <input className='checkbox' type='checkbox' onClick={onClick} checked={checked} onChange={(): void => {}} />
      <div className='hamburger-lines'>
        <span className='line line1'></span>
        <span className='line line2'></span>
        <span className='line line3'></span>
      </div>
    </>
  )
}

export { HamburgerButton }
