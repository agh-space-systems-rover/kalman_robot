import type { AxiosResponse } from 'axios'

type PromiseReturnType<P extends Promise<unknown>> = P extends Promise<infer T> ? T : never
type AsyncFnReturnType<F extends (...args: unknown[]) => Promise<unknown>> = PromiseReturnType<ReturnType<F>>

type AxiosFnReturnType<T> = Promise<AxiosResponse<T, unknown>>
