import { ArmPose } from './arm';

export interface PredefinedPoseConfig {
  max_distance_rad: number;
  stop_trajectory_timeout: number;
  poses: ArmPose[];
}

const PREDEFINED_POSES: PredefinedPoseConfig = {
  max_distance_rad: 0.35,
  stop_trajectory_timeout: 1.5,
  poses: [
    {
      id: 1,
      name: 'Based',
      joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    {
      id: 1,
      name: 'Compact Herman',
      joints: [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0]
    },
    {
      id: 0,
      name: 'Reset 4 and 6',
      isSpecial: true,
      joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    {
      id: 2,
      name: 'Base Front',
      joints: [0.0, 0.0125, 2.1765, 0.0, 0.7608, 0.0]
    }
  ]
};

export default PREDEFINED_POSES;