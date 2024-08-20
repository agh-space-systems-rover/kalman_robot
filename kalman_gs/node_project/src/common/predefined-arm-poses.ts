const PREDEFINED_POSES = {
  'max_distance_rad': 0.35,
  'stop_trajectory_timeout': 0.5,
  'poses': [
    {
      'id': 0,
      'name': 'base',
      'path': 'predefined_poses/move_group_goal.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [1, 2, 3]
    },
    {
      'id': 1,
      'name': 'left_base',
      'path': 'predefined_poses/left_base.yaml',
      'joints_set': [1, 2, 3, 4, 5, 6],
      'joints_checked': [1, 2, 3]
    },
    {
      'id': 2,
      'name': 'reset_4_and_6',
      'path': 'predefined_poses/reset.yaml',
      'joints_set': [4, 6],
      'joints_checked': []
    }
  ]
};

const POSES_JOINTS = {
  'base': [0.0, -0.4098, 2.567, 0.0, -0.6277, 0.0],
  'left_base': [0.9117, -0.2978, 2.2095, 0.1651, -0.5709, -0.1718],
  'reset_4_and_6': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
};

export default { PREDEFINED_POSES, POSES_JOINTS };
