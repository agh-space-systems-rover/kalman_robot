export enum TrajectoryType {
  Joints,
  Cartesian,
}

export interface Trajectory {
  label: string
  velocityDcalingFactor: number
  acceleration_scaling_factor: number
}

export interface Trajectories {
  cartesianSpaceGoals: Trajectory[]
  jointSpaceGoals: Trajectory[]
}

export interface TrajectoriesState {
  fetched: boolean
  trajectories: Trajectories
  newTrajectories: NewTrajectory[]
}

export type NewTrajectory = string
