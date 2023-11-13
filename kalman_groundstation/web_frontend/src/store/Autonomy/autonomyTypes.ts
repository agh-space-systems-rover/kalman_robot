import type { UspParams } from '../../components/AutonomyFront/Usp'

export interface AutonomyImu {
  alfa: number | null
  beta: number | null
  gamma: number | null
}

export interface AutonomyGps {
  position: {
    latitude: number | null
    longitude: number | null
    altitude: number | null
  }
}

export interface AutonomyPose {
  position: {
    x: number | null
    y: number | null
    z: number | null
  }
}

export interface AutonomyTag {
  id: string
  lat: number
  lon: number
}

export interface AutonomyIrcObject {
  id: number
  lat: number
  lon: number
  height: number
  heading: number
}

export interface AutonomyState {
  imu: AutonomyImu
  imuRaw: AutonomyImu
  ueuos: string
  gps: AutonomyGps
  odom: AutonomyPose
  map: AutonomyPose
  supervisorState: number
  tags: AutonomyTag[]
  usp?: UspParams | null
  waypoints?: number[] | null
  moveBaseStatus?: number | null
  bleSignal?: number | null
}
