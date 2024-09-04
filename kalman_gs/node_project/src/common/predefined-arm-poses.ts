const PREDEFINED_POSES = {
  'max_distance_rad': 0.35,
  'stop_trajectory_timeout': 1.5,
  'poses': [
    {
      'id': 0,
      'name': 'Compact Herman',
      'path': 'predefined_poses/compact_herman.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [1, 2, 3],
      'safe_previous_poses': [2, 3, 4, 5, 6]
    },
    {
      'id': 1,
      'name': 'Reset 4 and 6',
      'path': 'predefined_poses/reset.yaml',
      'joints_set': [4, 6],
      'joints_checked': [],
      'joints_reversed': [5],
      'safe_previous_poses': []
    },
    {
      'id': 2,
      'name': 'Base Front',
      'path': 'predefined_poses/base_front.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 3, 4, 5, 6]
    },
    {
      'id': 3,
      'name': 'Base Left',
      'path': 'predefined_poses/base_left.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 2, 4, 5]
    },
    {
      'id': 4,
      'name': 'Base Right',
      'path': 'predefined_poses/base_right.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 2, 3, 5]
    },
    {
      'id': 5,
      'name': 'Based',
      'path': 'predefined_poses/reset.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [],
      'safe_previous_poses': [0, 2, 3, 4, 5, 6]
    },
    {
      'id': 6,
      'name': 'Spectro approach',
      'path': 'predefined_poses/spectro_approach.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [2, 3],
      'safe_previous_poses': [0, 2, 5]
    }
  ]
};

const POSES_JOINTS = {
  'Compact Herman': [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0],
  'Reset 4 and 6': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  'Base Front': [0.0, 0.0125, 2.1765, 0.0, 0.7608, 0.0],
  'Base Left': [2.2435, 0.3388, 1.7765, 0.0, 1.0902, 0.7],
  'Base Right': [-2.2435, 0.3388, 1.7765, 0.0, 1.0902, -0.7],
  'Based': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  'Spectro approach': [0.0, 0.28, 2.4, 3.14, -1.7, 0.0]
};

export default { PREDEFINED_POSES, POSES_JOINTS };
