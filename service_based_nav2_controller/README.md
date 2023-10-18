# Service-Based Navigation2 Controller

Navigation2 FollowPath controller that calls a service to generate velocity commands.

## service_based_nav2_controller_srvs/srv/ComputeVelocityCommands Service

```
geometry_msgs/PoseStamped pose
geometry_msgs/Twist velocity
nav_msgs/Path path
---
geometry_msgs/TwistStamped cmd_vel
```

## Configuration

```
controller_server:
  ros__parameters:
    # ...
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "service_based_nav2_controller::ServiceBasedNav2Controller"
      compute_velocity_commands_service: "/your_own_path_follower/compute_velocity_commands"
```
