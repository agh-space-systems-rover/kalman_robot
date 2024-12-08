# Spacenav Wrapper

This package contains a launch file for the spacenav node. Which is essentially a driver for 3Dconnexion's SpaceMouse. Log messages from the node are throttled  flooding the console.

> [!NOTE]
> Sometimes, for the spacenav node to work, a daemon will have to be started using `sudo spacenavd` command.
> It should get started automatically by `kalman_ws` launch file, but if it doesn't, that's how you fix it.
