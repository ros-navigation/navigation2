# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors A plugin interface allows for the layers to be combined into the
costmap and finally inflated via a user specified inflation radius. The nav2 version of the costmap_2d package is mostly a direct
ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

## Overview of Changes from ROS1 Navigation Costmap_2d
- Removal of legacy parameter style ("Loading from pre-hydro parameter style")
- Intermediate replacement of dynamic reconfigure (not ported to ROS2). This discussion started here with costmap_2d but is a more
widespread discussion throughout the navigation stack (see issue https://github.com/ros-planning/navigation2/issues/177) and 
general ROS2 community. A proposal temporary replacement has been submitted as a PR here: https://github.com/ros-planning/navigation2/pull/196

## Future Plans
- Conceptually, the costmap_2d model acts as a world model of what is known from the map, sensor, robot pose, etc. We'd like
to broaden this world model concept and use costmap's layer concept as motivation for providing a service-style interface to
potential clients needing information about the world (see issue https://github.com/ros-planning/navigation2/issues/18)
