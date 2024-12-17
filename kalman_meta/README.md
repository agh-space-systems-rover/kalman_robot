# Meta Packages

This collection of packages does not implement any actual functionality, but rather declares dependencies on specific sets of packages, that are required together in a specific specific deployment scenario. Instead of building the whole workspace, the user can select a specific meta package from this collection to build it along with the packages required only for the selected scenario.

## Available Meta Packages

- `kalman_meta_arm`: arm control on the Raspberry Pi
- `kalman_meta_gs`: ground station laptop
- `kalman_meta_pc`: onboard PC with autonomous capabilities

## Usage Example

For deployment on the ground station laptop:

```bash
colcon build --packages-up-to kalman_meta_gs
```

This command would only build the ground station app and some communication drivers, skipping any autonomous subsystems and arm controllers.
