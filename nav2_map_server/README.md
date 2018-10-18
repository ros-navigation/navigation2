# Nav2 Map Server

The map server in the navigation2 stack is responsible for loading a map from an image file and providing a map message across 
the ROS2 Interface for potential clients. In comparison to the ROS1 navigation map server, the nav2 map server intends to support
a variety of potential map types, and thus some aspects of the original code have been refactored to support this extensible 
framework.

## Overview of changes from ROS1 Navigation Map Server
- Implementation of the Map Server has been refactored into an abstract base class for map server and its respective map type-specific 
implementations (i.e. nav_msgs/msg/OccupancyGrids)
- Loading and Parsing of YAML file (path currently provided as command line argument) and loading of the directed image file have
been refactored into function implementations from the map server abstract base class. 
- A factory pattern has been introduced to decide at launch time which map type to use
- It is optional whether the map is connected to the ROS Interface or is initialized as an object in another module (by
instantiating the map type-specific class)
- Map Saver not (yet) ported

## Currently Supported Map Types
- Occupancy Grid (nav_msgs/msg/OccupancyGrid)

## Future Plans
- Make abstract base class for map server derive from rclcpp::Node (https://github.com/ros-planning/navigation2/issues/189)
- Support new map types, e.g. GridMap (https://github.com/ros-planning/navigation2/issues/191)
- Load & Parse YAML file as ROS2 parameters via launch (https://github.com/ros-planning/navigation2/issues/190)
- Port and refactor Map Saver (https://github.com/ros-planning/navigation2/issues/188)
