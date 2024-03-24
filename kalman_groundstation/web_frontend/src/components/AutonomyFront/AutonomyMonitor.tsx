import type React from 'react'

import { useAppSelector } from '../../store/storeHooks'

function toMoveBaseStatus(movebaseStatus: number | null | undefined): string {
  switch (movebaseStatus) {
    case null:
      return 'unknown'
    case undefined:
      return 'unknown'
    case 0:
      return 'PENDING'
    case 1:
      return 'ACTIVE'
    case 2:
      return 'PREEMPTED'
    case 3:
      return 'SUCCEEDED'
    case 4:
      return 'ABORTED'
    case 5:
      return 'REJECTED'
    default:
      return 'unknown'
  }
}

const formatNumber: (value: number | null | undefined, precision: number) => string | number = (
  value: number | null | undefined,
  precision: number,
) => {
  if (value === null || value === undefined) {
    return 'unkown'
  } else {
    return value.toFixed(precision)
  }
}

interface AutonomyMonitorProps {
  headless?: boolean
}

export const AutonomyMonitor: React.FC<AutonomyMonitorProps> = ({ headless }) => {
  const autonomy = useAppSelector((state) => state.autonomy)

  // export interface AutonomyImu {
  //   alfa: number | null
  //   beta: number | null
  //   gamma: number | null
  // }

  // export interface AutonomyGps {
  //   position: {
  //     latitude: number | null
  //     longitude: number | null
  //   }
  // }

  // export interface AutonomyOdom {
  //   position: {
  //     x: number | null
  //     y: number | null
  //     z: number | null
  //   }
  // }

  // export interface AutonomyTag {
  //   id: string
  //   lat: number
  //   lon: number
  // }

  // export interface AutonomyIrcObject {
  //   id: number
  //   type: ircObjectType
  //   lat: number
  //   lon: number
  //   height: number
  //   heading: number
  // }

  // export interface UspParams {
  //   x?: number
  //   y?: number
  //   autonomousDriving?: boolean
  //   goalSet?: boolean
  //   multipleWaypoints?: boolean
  //   frame?: number
  //   mode?: number
  //   tag1?: number
  //   tag2?: number
  //   args?: string
  // }

  // export interface AutonomyState {
  //   imu: AutonomyImu
  //   imuRaw: AutonomyImu
  //   ueuos: string
  //   gps: AutonomyGps
  //   odom: AutonomyOdom
  //   supervisorState: number
  //   tags: AutonomyTag[]
  //   ircObjects: AutonomyIrcObject[]
  //   usp?: UspParams | null
  //   waypoints?: number[] | null
  //   moveBaseStatus?: number | null
  // }

  return headless ? (
    <></>
  ) : (
    <div style={{ overflow: 'scroll' }}>
      <h3>Autonomy State</h3>
      <div>Imu: {(((autonomy.imu.gamma || 0) * 180) / Math.PI).toFixed(2)}</div>
      <div> Ueuos: {autonomy.ueuos} </div>
      <div>
        GPS: {formatNumber(autonomy.gps.position.latitude, 6)}, {formatNumber(autonomy.gps.position.longitude, 6)}
      </div>
      <div>
        Odom: {formatNumber(autonomy.odom.position.x, 2)}, {formatNumber(autonomy.odom.position.y, 2)}
      </div>
      <div>
        Map: {formatNumber(autonomy.map.position.x, 2)}, {formatNumber(autonomy.map.position.y, 2)}
      </div>
      <div>Supervisor State: {autonomy.supervisorState}</div>
      <div>
        Tags:{' '}
        <ul>
          {autonomy.tags.map((tag) => (
            <li key={tag.id}>
              • {tag.id} ({formatNumber(tag.lat, 6)}, {formatNumber(tag.lon, 6)})
            </li>
          ))}
        </ul>
      </div>
      <div>
        Waypoints:{' '}
        <ul>
          {autonomy.waypoints?.map((wp) => (
            <li key={wp}>• {wp}</li>
          ))}
        </ul>
      </div>
      <div>Move Base Status: {toMoveBaseStatus(autonomy.moveBaseStatus)}</div>
    </div>
  )
}
