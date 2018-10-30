# Map Server

The `Map Server` is responsible to provide maps to the rest of the nav2 system using 
both topic and service interfaces. In contrast to the ROS1 navigation map server, the nav2 map 
server will support a variety of map types, and thus some aspects of the original code have been 
refactored to support this new extensible framework.

## Overview of changes from ROS1 Navigation Map Server
- The nav1 Map Server code has been refactored into a `MapLoader` abstract base class, type-specific map loaders such as an `OccGridLoader`, and a `MapServer` class that manages the loaders
- A map loader loads an image file, compose a ROS message for the loaded map, and present the result via topic and service interfaces
- Direct loading and parsing of a YAML configuration file has been removed in favor of using ROS2 parameters
A ROS2-style YAML file can still be specified at launch time, which provides an opportunity to override default parameter values, which are set in the code

## Currently Supported Map Types
- Occupancy grid (nav_msgs/msg/OccupancyGrid), via the OccGridLoader

## Future Plans
- Support additional map types, e.g. GridMap (https://github.com/ros-planning/navigation2/issues/191)
- Port and refactor Map Saver (https://github.com/ros-planning/navigation2/issues/188)
