# kalman_description

Xacro / URDF descriptions + models for the rover

## Launch Arguments

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_state_publisher_gui",
                default_value="true",
                description="Start the joint state publisher in GUI mode.",
            ),
            DeclareLaunchArgument(
                "with_arm",
                default_value="false",
                description="Set robot description to include the arm.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


- `joint_state_publisher_gui`: Start the joint state publisher in GUI mode.
- `with_arm`: Set robot description to include the arm.