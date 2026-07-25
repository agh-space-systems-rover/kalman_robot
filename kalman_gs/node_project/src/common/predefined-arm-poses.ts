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
      'name': 'Reset 4 and 6',
      'path': 'predefined_poses/reset.yaml',
      'joints': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      'joints_set': [4, 6],
      'joints_checked': [],
      'joints_reversed': [5],
      'safe_previous_poses': []
    },
    {
      'id': 2,
      'name': 'Base Front',
      'path': 'predefined_poses/base_front.yaml',
      'joints': [0.0, 0.0125, 2.1765, 0.0, 0.7608, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 3, 4, 5, 6]
    },
    {
      'id': 3,
      'name': 'Base Left',
      'path': 'predefined_poses/base_left.yaml',
      'joints': [2.2435, 0.3388, 1.7765, 0.0, 1.0902, 0.7],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 2, 4, 5]
    },
    {
      'id': 4,
      'name': 'Base Right',
      'path': 'predefined_poses/base_right.yaml',
      'joints': [-2.2435, 0.3388, 1.7765, 0.0, 1.0902, -0.7],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 2, 3, 5]
    },
    {
      'id': 5,
      'name': 'Based',
      'path': 'predefined_poses/reset.yaml',
      'joints': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 3, 4, 5, 6]
    },
    {
      'id': 6,
      'name': 'Sand Storage',
      'path': 'predefined_poses/sand_storage.yaml',
      'joints': [-2.47, 0.08, 1.69, 0.6, 0.98, -0.88],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 5]
    },
    {
      'id': 7,
      'name': 'Rock Storage',
      'path': 'predefined_poses/rock_storage.yaml',
      'joints': [-3.14, -0.7, 2.2, 0.0, 1.0, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 5]
    },
    {
      'id': 8,
      'name': 'Spectro Approach',
      'path': 'predefined_poses/spectro_approach.yaml',
      'joints': [0.0, 0.28, 2.4, 3.14, -1.7, 0.0],
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 5]
    }
  ]
};
export default PREDEFINED_POSES;
