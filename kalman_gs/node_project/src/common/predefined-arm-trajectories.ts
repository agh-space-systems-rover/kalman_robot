const PREDEFINED_TRAJECTORIES = {
  'max_distance_rad': 0.05,
  'stop_trajectory_timeout': 0.5,
  'trajectories': [
    {
      'id': 0,
      'name': 'goal',
      'path': 'predefined_trajectories/traj1.yaml'
    },
    {
      'id': 1,
      'name': 'science_goal',
      'path': 'predefined_trajectories/science_goal.yaml'
    }
  ]
};

const START_JOINTS = {
  'goal': [-1.1, -0.4, 2.1, 0.0, -1.7, 1.1],
  'science_goal': [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0]
};

export default { PREDEFINED_TRAJECTORIES, START_JOINTS };
