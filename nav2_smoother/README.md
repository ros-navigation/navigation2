# Nav2 Smoother

The Nav2 smoother is a Task Server in Nav2 that implements the `nav2_behavior_tree::SmoothPath` interface.

A smoothing module implementing the `nav2_behavior_tree::SmoothPath` interface is responsible for improving path smoothness and/or quality, typically given an unsmoothed path from the planner module in `nav2_planner`. It loads a map of potential smoother plugins to do the path smoothing in different user-defined situations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available smoother plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-smoother-server.html) for additional parameter descriptions.

This package contains the Simple Smoother and Savitzky-Golay Smoother plugins.
