export interface SequenceManagerState {
  state: number
  stage: number
}

export interface ScienceUniversal {
  leds: number[]
  heatingUp: number
  heatingDown: number
  washer: number
  sequenceStates: SequenceManagerState[]
}

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
  universal: ScienceUniversal
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

export const mapScienceUniversal = (msg: any): ScienceUniversal => {
  return {
    leds: [msg.led1.data, msg.led2.data, msg.led3.data],
    heatingDown: msg.heating_down.data,
    heatingUp: msg.heating_up.data,
    sequenceStates: [
      { stage: msg.seq_stage1, state: msg.seq_state1 },
      { stage: msg.seq_stage2, state: msg.seq_state2 },
      { stage: msg.seq_stage3, state: msg.seq_state3 },
    ],
    washer: msg.washer.data,
  }
}
