**Warning**: As with the rest of `nav2`, this package is still in development and only works with Turtlebot 3 at the moment. Currently collision avoidance has not been integrated. The user is advised to not use this feature on a physical robot for safety reasons.  As of now, this feature should only be used in simulations.

---

# Default Recoveries

The `nav2_default_recoveries` package implements a module for executing simple controlled robot movements such as rotating on its own axis or moving linearly.

The package defines the `BackUp`, `Spin` and `Stop` recoveries.

## Overview

Currently the package provides the following recoveries:
- **Spin** performs an in-place rotation by a given angle.
- **Back Up** performs an linear translation by a given distance.
- **Stop** brings the robot to a stationary state.
