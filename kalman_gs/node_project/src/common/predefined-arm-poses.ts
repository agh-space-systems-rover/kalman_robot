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
    },
    {
      id: 3,
      name: 'Base Left',
      joints: [2.2435, 0.3388, 1.7765, 0.0, 1.0902, 0.7]
    },
    {
      id: 4,
      name: 'Base Right',
      joints: [-2.2435, 0.3388, 1.7765, 0.0, 1.0902, -0.7]
    },
    {
      id: 5,
      name: 'Based',
      joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    {
      id: 6,
      name: 'Sand Storage',
      joints: [-2.47, 0.08, 1.69, 0.6, 0.98, -0.88]
    },
    {
      id: 7,
      name: 'Rock Storage',
      joints: [-3.14, -0.7, 2.2, 0.0, 1.0, 0.0]
    },
    {
      id: 8,
      name: 'Spectro Approach',
      joints: [0.0, 0.28, 2.4, 3.14, -1.7, 0.0]
    }
  ]
};

export default PREDEFINED_POSES;