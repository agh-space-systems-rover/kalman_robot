import type { Action } from '@reduxjs/toolkit'
import type { Dispatch } from 'react'
import type { TypedUseSelectorHook } from 'react-redux'
import { useDispatch, useSelector } from 'react-redux'

import type { AppDispatch, RootState } from './store'

// Use throughout your app instead of plain `useDispatch` and `useSelector`
export const useAppDispatch: () => Dispatch<Action<unknown>> = () => useDispatch<AppDispatch>()
export const useAppSelector: TypedUseSelectorHook<RootState> = useSelector
