import type { AxiosRequestHeaders, AxiosResponse } from 'axios'
import axios from 'axios'

import { store } from '../store/store'

/**
 *
 * @param endpoint api endpoint beginning with "/", omit "/api/v1"
 * @returns api request Promise
 */
export const getRequest = (
  endpoint: string,
  headers?: AxiosRequestHeaders,
): Promise<AxiosResponse<unknown, unknown>> => {
  const backUrl = store.getState().settings.backUrl
  const request = axios.get(`http://${backUrl}${endpoint}`, { headers })
  return request
}

/**
 *
 * @param endpoint api endpoint beginning with "/", omit "/api/v1"
 * @returns api request Promise
 */
export const postRequest = (
  endpoint: string,
  data?: object,
  headers?: AxiosRequestHeaders,
): Promise<AxiosResponse<unknown, unknown>> => {
  const backUrl = store.getState().settings.backUrl
  const request = axios.post(`http://${backUrl}${endpoint}`, data, { headers })
  return request
}

/**
 *
 * @param endpoint api endpoint beginning with "/", omit "/api/v1"
 * @returns api request Promise
 */
export const putRequest = (
  endpoint: string,
  data?: object,
  headers?: AxiosRequestHeaders,
): Promise<AxiosResponse<unknown, unknown>> => {
  const backUrl = store.getState().settings.backUrl
  const request = axios.put(`http://${backUrl}${endpoint}`, data, { headers })
  return request
}
