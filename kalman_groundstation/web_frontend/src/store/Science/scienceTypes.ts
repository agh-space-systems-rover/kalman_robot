export interface Module {
  temperature: number
}

export interface Atmosphere {
  temperature: number
  humidity: number
  pressure: number
}

export interface SmartProbe {
  temperature: number
  hummidity: number
}

export interface ScienceState {
  atmosphere: Atmosphere
  modules: Module[]
  cameraFeedback: string
  cameraFeedbackTime: number
  ERCTemperature: number
  ERCWeight: number
  smartProbe: SmartProbe
  panoramaStatus: PanoramaStatus
}

export enum PanoramaStatus {
  INITIAL = -1,
  WAITING = 0,
  SUCCES = 1,
  FAILURE = 2,
}

export const mapScience = (msg: any): Module[] => {
  return [
    {
      temperature: msg.temp_slot_1.data,
    },
    {
      temperature: msg.temp_slot_2.data,
    },
    {
      temperature: msg.temp_slot_3.data,
    },
  ]
}

export const mapSmartProbe = (msg: any): SmartProbe => {
  return {
    temperature: msg.temperature,
    hummidity: msg.humidity,
  }
}
