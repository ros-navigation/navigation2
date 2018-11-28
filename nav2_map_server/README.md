# Map Server

The `Map Server` provides maps to the rest of the Navigation2 system using both topic and
service interfaces. 

## Changes from ROS1 Navigation Map Server

While the nav2 map server provides the same general function as the nav1 map server, the new
code has some changes to accomodate ROS2 as well as some architectural improvements.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support 
this new extensible framework. In particular, there is now a `MapLoader` abstract base class 
and type-specific map loaders which derive from this class. There is currently one such
derived class, the `OccGridLoader`, which converts an input image to an OccupancyGrid and
makes this available via topic and service interfaces. The `MapServer` class is a ROS2 node
that uses the appropriate loader, based on an input parameter.

### ROS2 Node

The Map Server is a composable ROS2 node. By default, there is a map_server executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

### Command-line arguments, ROS2 Node Parameters, and YAML files

Because the map server is now a composable node, the command line for the map server is different
that it was with ROS1. With ROS1, one invoked the map server, passing the map YAML filename:

```
$ ./map_server map.yaml
```

Where the YAML file specified the various parameters. For example, 

```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

With ROS2 and node composition, node parameters come through a parameter file, which can
contain parameters for one or more nodes. For example, for a node named 'map_server', the 
parameter file could look like this:

```
map_server:
    ros__parameters:
        <params>
```

With nav1, some of these parameters specified in the map YAML file are map-specific 
metadata (resolution, origin), while others are parameters for the conversion process (free threshold,
occupied threshold, etc.). These parameters have been split into two files. The first file is like
ROS1 where the map-specific metadata is in a ROS1-like YAML file and, for convenience, can physically
reside alongside the map file. For example,

```
# Example YAML file that resides with the image (testmap.yaml)
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
```

The other file contains the parameters for the Map Server which configure the loader. For example,

```
# Example ROS2 configuration file that configures the map server node (map_server_params.yaml)
map_server:
    ros__parameters:
        yaml_filename: "testmap.yaml"
        map_type: "occupancy"
        free_thresh: 0.196
        occupied_thresh: 0.65
        mode: "trinary"
        negate: 0
```

## Currently Supported Map Types
- Occupancy grid (nav_msgs/msg/OccupancyGrid), via the OccGridLoader

## Future Plans
- Allow for dynamic configuration of conversion parameters
- Support additional map types, e.g. GridMap (https://github.com/ros-planning/navigation2/issues/191)
- Port and refactor Map Saver (https://github.com/ros-planning/navigation2/issues/188)
