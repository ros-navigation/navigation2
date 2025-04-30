# nav2_toolkit

A modular ROS 2 node for persisting and restoring the robot's last known pose using the `/amcl_pose` topic. This is useful in applications where the robot is never manually moved (e.g. warehouse robots), and it enables automatic re-localization.

## Features
- Save the current AMCL pose to disk periodically
- Write safely using an atomic temp-file swap
- Restore the robot pose automatically on command
- Start/Stop pose saving dynamically with services
- Reset pose file to zeros (e.g. to clear old pose data)
- Parameter configurable save interval and file path
- Support for saving to multiple locations
