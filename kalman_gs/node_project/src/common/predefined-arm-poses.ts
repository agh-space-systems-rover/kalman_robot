const PREDEFINED_POSES = {
  'max_distance_rad': 0.35,
  'stop_trajectory_timeout': 1.5,
  'poses': [
    {
      'id': 0,
      'name': 'Compact Herman',
      'path': 'predefined_poses/compact_herman.yaml',
      'joints': [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [1, 2, 3],
      'safe_previous_poses': [2, 3, 4, 5, 6]
    },
    {
      'id': 1,
      'name': 'Based',
      'path': 'predefined_poses/reset.yaml',
      'joints': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 3, 4, 5, 6]
    }
  ]
};
export default PREDEFINED_POSES;
