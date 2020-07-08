# Map Server

The `Map Server` provides maps to the rest of the Navigation2 system using both topic and
service interfaces.

## Changes from ROS1 Navigation Map Server

While the nav2 map server provides the same general function as the nav1 map server, the new
code has some changes to accomodate ROS2 as well as some architectural improvements.

In addition, there is now a new "load_map" service which can be used to dynamically load a map.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework. In particular, there is now a `MapLoader` abstract base class
and type-specific map loaders which derive from this class. There is currently one such
derived class, the `OccGridLoader`, which converts an input image to an OccupancyGrid and
makes this available via topic and service interfaces. The `MapServer` class is a ROS2 node
that uses the appropriate loader, based on an input parameter.

### Command-line arguments, ROS2 Node Parameters, and YAML files

The Map Server is a composable ROS2 node. By default, there is a map_server executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1.
With ROS1, one invoked the map server and passing the map YAML filename, like this:

```
$ map_server map.yaml
```

Where the YAML file specified contained the various map metadata, such as:

```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Navigation2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a
level of indirection to get the map yaml filename. For example, for a node named 'map_server',
the parameter file would look like this:

```
# map_server_params.yaml
map_server:
    ros__parameters:
        yaml_filename: "map.yaml"
```

One can invoke the map service executable directly, passing the params file on the command line,
like this:

```
$ map_server __params:=map_server_params.yaml
```

There is also possibility of having multiple map server nodes in a single process, where the parameters file would separate the parameters by node name, like this:

```
# combined_params.yaml
map_server1:
    ros__parameters:
        yaml_filename: "some_map.yaml"

map_server2:
    ros__parameters:
        yaml_filename: "another_map.yaml"
```

Then, one would invoke this process with the params file that contains the parameters for both nodes:

```
$ process_with_multiple_map_servers __params:=combined_params.yaml
```

## Currently Supported Map Types
- Occupancy grid (nav_msgs/msg/OccupancyGrid), via the OccGridLoader

## Services
As in ROS navigation, the map_server node provides a "map" service to get the map. See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, map_server also now provides a "load_map" service. See nav2_msgs/srv/LoadMap.srv for details.

Example:
```
$ ros2 service call /load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}
```

