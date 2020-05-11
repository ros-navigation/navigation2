# Recovery Plugins

The `nav2_recovery_plugins` package implements a module for executing simple controlled robot movements such as rotating on its own axis or moving linearly.

The package defines the `BackUp`, `Spin` and `Stop` recoveries.

## Overview

Currently the package provides the following recoveries:
- **Spin** performs an in-place rotation by a given angle.
- **Back Up** performs an linear translation by a given distance.
- **Stop** brings the robot to a stationary state.
