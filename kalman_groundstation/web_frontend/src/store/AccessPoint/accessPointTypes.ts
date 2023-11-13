export interface AccessPointState {
  ssid: string
  status: string
  content: string
}

export const initialAccessPointState: AccessPointState = {
  ssid: '',
  status: '',
  content: '',
}
