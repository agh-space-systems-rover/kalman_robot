# RViz Configs

This package contains multiple RViz layout configuration files. There's a single launch file that can start one or multiple instances of RViz with those configurations.

## Usage

```bash
ros2 launch kalman_rviz rviz.launch.py configs:="autonomy arm1 arm2" joint_state_publisher_gui:=true
```

Where `configs` is a space-separated set of RViz configuration file names without extensions, e.g. 'autonomy demo_rgbd'. In this case the respective layouts would be located in `./rviz/autonomy.rviz` and `./rviz/demo_rgbd.rviz`.

New layouts can be added in the [rviz](./rviz) directory. The launch file won't need any changes to include the added layouts.
