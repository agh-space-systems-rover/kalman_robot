import type { AxiosResponse } from 'axios'

import type { UspParams } from '../components/AutonomyFront/Usp'
import type { AxisLockConfig } from '../features/keybindings/keybindingsTypes'
import { store } from '../store/store'
import { setNewTrajectories, setTrajectories } from '../store/Trajectories/trajectoriesSlice'
import type { NewTrajectory, Trajectories, Trajectory } from '../store/Trajectories/trajectoriesTypes'
import { TrajectoryType } from '../store/Trajectories/trajectoriesTypes'
import { getRequest, putRequest } from './axios'

export const sendUsp: (params: UspParams) => void = (params: UspParams) => {
  const request = putRequest(
    `/station/system/rover/autonomy/set_usp/?${Object.entries(params)
      .map(([k, v]) => `${k}=${v}&`)
      .join('')}`,
  )
  request.then((res) => console.log(res))
}

export const sendAutonomy: (autonomyOn: boolean) => void = (autonomyOn: boolean) => {
  const request = putRequest(`/station/system/rover/autonomy/autonomy_on_off/?autonomy_on=${autonomyOn}`)
  request.then((res) => console.log(res))
}

export const sendSilentMode: (silentOn: boolean) => void = (silentOn: boolean) => {
  const request = putRequest(`/station/system/radio/silent_mode_on_off/?silent_on=${silentOn}`)
  request.then((res) => console.log(res))
}

export const sendWaypointsSet: (waypoints: [number, number][]) => void = (waypoints: [number, number][]) => {
  const request = putRequest('/station/system/rover/autonomy/waypoints/set/', waypoints.flat())
  request.then((res) => console.log(res))
}

export const sendWaypointsGet: () => void = () => {
  const request = putRequest('/station/system/rover/autonomy/waypoints/get')
  request.then((res) => console.log(res))
}

export const sendUspRequest: () => void = () => {
  const request = putRequest('/station/system/rover/autonomy/get_usp')
  request.then((res) => console.log(res))
}

export const setDimensionLock: ({ lockX, lockY, lockZ, lockRotX, lockRotY, lockRotZ }: AxisLockConfig) => void = ({
  lockX,
  lockY,
  lockZ,
  lockRotX,
  lockRotY,
  lockRotZ,
}: AxisLockConfig) => {
  const request = putRequest(
    // eslint-disable-next-line max-len
    `/station/system/rover/arm/configuration/control_dimensions?control_x_translation=${lockX}&control_y_translation=${lockY}&control_z_translation=${lockZ}&control_x_rotation=${lockRotX}&control_y_rotation=${lockRotY}&control_z_rotation=${lockRotZ}`,
  )
  request.then((res) => console.log(res))
}

export const resetServoServer: () => void = () => {
  const request = putRequest('/station/system/rover/arm/configuration/reset_servo_status')
  request
    .then(() => console.log('Servo status successfully reset'))
    .catch(() => console.error('Unable to reset servo server status'))
}

export const getTrajectories: () => void = () => {
  const request = getRequest('/station/system/rover/arm/goal/') as Promise<AxiosResponse<Trajectories, unknown>>
  request
    .then((res: AxiosResponse<Trajectories, unknown>) => {
      store.dispatch(setTrajectories(res.data))
    })
    .catch(() => {
      store.dispatch(setTrajectories({ cartesianSpaceGoals: [], jointSpaceGoals: [] }))
    })
}

export const getNewTrajectories: () => void = () => {
  const request = getRequest('/station/system/rover/arm/trajectories/') as Promise<
    AxiosResponse<NewTrajectory[], unknown>
  >
  request.then((res: AxiosResponse<NewTrajectory[], unknown>) => {
    store.dispatch(setNewTrajectories(res.data))
  })
}

export const executeNewTrajectory: (name: string) => void = (name: string) => {
  const request = putRequest(`/station/system/rover/arm/trajectories/execute?name=${name}`)
  request.then((r) => console.log(r))
}

export const abortNewTrajectories: () => void = () => {
  const request = putRequest('/station/system/rover/arm/trajectories/abort')
  request.then((r) => console.log(r))
}

export const planTrajectory: (trajectory: Trajectory, type: TrajectoryType) => void = (
  trajectory: Trajectory,
  type: TrajectoryType,
) => {
  switch (type) {
    case TrajectoryType.Cartesian:
      putRequest('/station/system/rover/arm/goal/cartesian/plan/', trajectory).catch((e) => console.log(e))
      break
    case TrajectoryType.Joints:
      putRequest('/station/system/rover/arm/goal/joints/plan/', trajectory).catch((e) => console.log(e))
      break
  }
}

export const executeTrajectory: (trajectory: Trajectory, type: TrajectoryType) => void = (
  trajectory: Trajectory,
  type: TrajectoryType,
) => {
  switch (type) {
    case TrajectoryType.Cartesian:
      putRequest('/station/system/rover/arm/goal/cartesian/execute/', trajectory).catch((e) => console.log(e))
      break
    case TrajectoryType.Joints:
      putRequest('/station/system/rover/arm/goal/joints/execute/', trajectory).catch((e) => console.log(e))
      break
  }
}

export const setArmLinearVelocity: (scale: number) => void = (scale: number) => {
  putRequest(`/station/system/rover/arm/configuration/linear_vel_scale?scale=${scale}`).catch((e) => console.log(e))
}

export const setArmAngularVelocity: (scale: number) => void = (scale: number) => {
  putRequest(`/station/system/rover/arm/configuration/angular_vel_scale?scale=${scale}`).catch((e) => console.log(e))
}

export const startPositioning: (jointId: number) => void = (jointId: number) => {
  putRequest(`/station/system/rover/arm/positioning/start?joint_id=${jointId}`).catch((e) => console.log(e))
}

export const abortPositioning: () => void = () => {
  putRequest('/station/system/rover/arm/positioning/abort/').catch((e) => console.log(e))
}

export const setActualJointPosition: (jointId: number, positionRadians: number) => void = (
  jointId: number,
  positionRadians: number,
) => {
  putRequest(
    // eslint-disable-next-line max-len
    `/station/system/rover/arm/positioning/set_actual_joint_position?joint_id=${jointId}&position_radians=${positionRadians}`,
  ).catch((e) => console.log(e))
}

export const setCamera: (feed: number, camera: number) => void = (feed: number, camera: number) => {
  putRequest(`/station/system/rover/video/configuration/camera?feed=${feed}&camera=${camera}`).catch((e) =>
    console.log(e),
  )
}

export const setChannel: (dupa: boolean, feed: number, channel: number) => void = (
  dupa: boolean,
  feed: number,
  channel: number,
) => {
  putRequest(`/station/system/rover/video/configuration/channel?dupa=${dupa}&feed=${feed}&channel=${channel}`).catch(
    (e) => console.log(e),
  )
}

export const setPower: (feed: number, power: number) => void = (feed: number, power: number) => {
  putRequest(`/station/system/rover/video/configuration/power?feed=${feed}&power=${power}`).catch((e) => console.log(e))
}

export const pcPower: () => void = () => {
  putRequest('/station/system/rover/master/configuration/pc_power').catch((e) => console.log(e))
}

export const setMaxVelocityAtRover: (velocity: number) => void = (velocity: number) => {
  putRequest(`/station/system/rover/autonomy/max_velocity?speed=${velocity}`).catch((e) => console.log(e))
}

export const sendYawOffset: (offset: number) => void = (offset: number) => {
  putRequest(`/station/system/rover/autonomy/yaw_offset?offset=${offset}`).catch((e) => console.log(e))
}

export const sendUeuosState: (color: number) => void = (color: number) => {
  putRequest(`/station/system/rover/autonomy/ueuos_state?color=${color}`).catch((e) => console.log(e))
}

export const clearCostmap: () => void = () => {
  putRequest('/station/system/rover/autonomy/clear_costmap').catch((e) => console.log(e))
}
export const requestCostmap: () => void = () => {
  putRequest('/station/system/rover/autonomy/get_costmap').catch((e) => console.log(e))
}

export const setSciencePump: (slot: number, value: boolean) => void = (slot, value) => {
  putRequest(`/station/system/rover/science/set_pump?slot=${slot}&value=${value}`).catch((e) => console.log(e))
}

export const setScienceHeater: (slot: number, value: boolean) => void = (slot, value) => {
  putRequest(`/station/system/rover/science/set_heater?slot=${slot}&value=${value}`).catch((e) => console.log(e))
}

export const setScienceBacklight: (slot: number, value: boolean) => void = (slot, value) => {
  putRequest(`/station/system/rover/science/set_backlight?slot=${slot}&value=${value}`).catch((e) => console.log(e))
}

export const setScienceAux: (slot: number, value: boolean) => void = (slot, value) => {
  putRequest(`/station/system/rover/science/set_aux?slot=${slot}&value=${value}`).catch((e) => console.log(e))
}

export const setScienceServo: (slot: number, angle_deg: number) => void = (slot, angleDeg) => {
  putRequest(`/station/system/rover/science/set_servo?slot=${slot}&angle_deg=${angleDeg}`).catch((e) => console.log(e))
}

export const setAutoclick: (on: boolean) => void = (on) => {
  const value = on ? 255 : 0
  putRequest(`/station/system/rover/arm/autoclick/autoclick?value=${value}`).catch((e) => console.log(e))
}

export const setCarousel: (value: number) => void = (value: number) => {
  putRequest(`/station/system/rover/science/set_carousel?value=${value}`).catch((e) => console.log(e))
}

export const setCarouselWithOffset: (value: number, offset: number) => void = (value: number, offset: number) => {
  putRequest(`/station/system/rover/science/set_carousel_with_offset?value=${value}&offset=${offset}`).catch((e) =>
    console.log(e),
  )
}

export const openCloseSampleERC: (open: boolean) => void = (open: boolean) => {
  putRequest(`/station/system/rover/science/open_close_sample?open=${open}`).catch((e) => console.log(e))
}

export const requestWeightERC: () => void = () => {
  putRequest('/station/system/rover/science/get_weight').catch((e) => console.log(e))
}

export const tareWeight: () => void = () => {
  putRequest('/station/system/rover/science/tare_weight').catch((e) => console.log(e))
}

export const requestPanorama: (lat: number, lon: number, alt: number) => void = (lat, lon, alt) => {
  putRequest(`/station/system/rover/science/get_panorama?lat=${lat}&lon=${lon}&alt=${alt}`).catch((e) => console.log(e))
}

export const requestProbeMeasurement: () => void = () => {
  putRequest('/station/system/rover/science/get_smart_probe').catch((e) => console.log(e))
}

export const logUserMark: (lat: number, lon: number, alt: number, desc: string) => void = (lat, lon, alt, desc) => {
  putRequest(`/station/system/rover/science/log_user_marker?lat=${lat}&lon=${lon}&alt=${alt}&desc=${desc}`).catch((e) =>
    console.log(e),
  )
}

export const requestLampPWM: (value: number) => void = (value) => {
  putRequest(`/station/system/rover/science/lamp_pwm?value=${value}`).catch((e) => console.log(e))
}

export const requestLEDState: (channel: number, value: number) => void = (channel, value) => {
  putRequest(`/station/system/rover/science/led_state?channel=${channel}&value=${value}`).catch((e) => console.log(e))
}

export const requestHBridgeState: (channel: number, speed: number, direction: number) => void = (
  channel,
  speed,
  direction,
) => {
  putRequest(
    `/station/system/rover/science/set_h_bridge?channel=${channel}&speed=${speed}&direction=${direction}`,
  ).catch((e) => console.log(e))
}

export const requestPWMState: (channel: number, value: number) => void = (channel, value) => {
  putRequest(`/station/system/rover/science/set_pwm?channel=${channel}&value=${value}`).catch((e) => console.log(e))
}

export const requestSequenceBegin: (sequenceId: number) => void = (sequenceId: number) => {
  putRequest(`/station/system/rover/science/sequence_begin?sequence_id=${sequenceId}`).catch((e) => console.log(e))
}

export const requestSequenceState: (sequenceId: number) => void = (sequenceId: number) => {
  putRequest(`/station/system/rover/science/sequence_state?sequence_id=${sequenceId}`).catch((e) => console.log(e))
}
