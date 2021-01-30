# Map Server

The `Map Server` provides maps to the rest of the Nav2 system using both topic and
service interfaces.

## Changes from ROS1 Navigation Map Server

While the nav2 map server provides the same general function as the nav1 map server, the new
code has some changes to accommodate ROS2 as well as some architectural improvements.

In addition, there is now two new "load_map" and "save_map" services which can be used to
dynamically load and save a map, and the new map server supports `3D point-clouds` via 
template metaprogramming.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework.

Currently, map server divides into four parts:

- `map_server`
- `map_saver`
- `map_io_2d` library
- `map_io_3d` library

Detailed library structure:
```c++
--------------------------------------------------------------------
--------------------------------------------------------------------
|namespace nav2_map_server
||-> map_server #include "nav2_map_server/map_server.hpp"
||     |-> MapServer<nav_msgs::msg::OccupancyGrid>
||     |        |-> service: get_map "nav_msgs/srv/GetMap.srv"
||     |        |-> service: load_map "nav2_msgs/srv/LoadMap.srv"
||     |-> MapServer<sensor_msgs::msg::PointCloud2>
||     |        |-> service: get_map "nav2_msgs/srv/GetMap3D.srv"
||     |        |-> service: load_map "nav2_msgs/srv/LoadMap3D.srv"
||-> map_saver "nav2_map_server/map_saver.hpp"
||     |-> MapSaver<nav_msgs::msg::OccupancyGrid>
||     |        |-> service: save_map "nav2_msgs/srv/SaveMap.srv"
||     |-> MapSaver<sensor_msgs::msg::PointCloud2>
||     |        |-> service: save_map "nav2_msgs/srv/SaveMap3D.srv"
||------------------------------------------------------------------
||-> namespace map_2d #include "nav2_map_server/map_2d/map_io_2d.hpp"
||               |-> struct LoadParameters
||               |-> enum class LOAD_MAP_STATUS
||               |-> LoadParameters loadMapYaml(...)
||               |-> void loadMapFromFile(...)
||               |-> LOAD_MAP_STATUS loadMapFromYaml(...)
||               |-> struct SaveParameters
||               |-> bool saveMapToFile(...)
||-> namespace map_3d #include "nav2_map_server/map_3d/map_io_3d.hpp"
||               |-> struct LoadParameters
||               |-> enum class LOAD_MAP_STATUS
||               |-> LoadParameters loadMapYaml(...)
||               |-> void loadMapFromFile(...)
||               |-> LOAD_MAP_STATUS loadMapFromYaml(...)
||               |-> struct SaveParameters
||               |-> bool saveMapToFile(...)
--------------------------------------------------------------------
--------------------------------------------------------------------
```

`map_server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`map_saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`map_io_2d` - is a map input-output library for `occupancy-grids`. The library is designed to be an object-independent
in order to allow easily save/load map from external code just by calling necessary function.
This library is also used by `map_server<nav_msgs::msg::OccupancyGrid>`
, and `map_saver<nav_msgs::msg::OccupancyGrid>`
to work. It supports
OccupancyGrid saving/loading functions moved from the rest part of map server code.
It is designed to be replaceable for a new IO library (e.g. for library with a new map encoding
method or any other library supporting costmaps, multifloor maps, etc...).

`map_io_3D` - is a map input-output library for `3D PointClouds` and deals especially with `.pcd` 
file I/O for more details about `pcd` file format please refer to 
[pcl website](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html#pcd-file-format).
 The library is used by `map_server<sensor_msgs::msg::PointCloud2>`, and `map_saver<sensor_msgs::msg::PointCloud2>`.

### CLI-usage

#### Map Server

The `Map Server` is a composable ROS2 node. By default, there is a `map_server_2d` and `map_server_3d` executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1.
With ROS1, one invoked the map server and passing the map YAML filename, like this:

```
$ map_server_2d map_2d.yaml
```

Where the YAML file specified contained the various map metadata, such as:

- for an `image` as input/output
```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

```
$ map_server_3d map_3d.yaml
```

For 3D map server YAML can be:

- for a `pcd` as input/output 
```
image: testmap.pcd
origin: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
```

The Navigation2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a
level of indirection to get the map yaml filename. For example, for a node named 'map_server',
the parameter file would look like this:

```yaml
# map_server_params_2d.yaml
map_server_2d:
    ros__parameters:
        yaml_filename: "map_2d.yaml"
```
```yaml
# map_server_params_3d.yaml
map_server_3d:
    ros__parameters:
        yaml_filename: "map_3d.yaml"
```

One can invoke the map service executable directly, passing the params file on the command line,
like this:

```shell
# For 2D server
map_server_2d __params:=map_server_params_2d.yaml
```
```shell
# For 3D server
map_server_3d __params:=map_server_params_3d.yaml
```

There is also possibility of having multiple map server nodes in a single process, where the parameters file would separate the parameters by node name, like this:

```yaml
# For 2D map server
# combined_params_2d.yaml
map_server_2d1:
    ros__parameters:
        yaml_filename: "some_map_2d.yaml"

map_server_2d2:
    ros__parameters:
        yaml_filename: "another_map_2d.yaml"
```
```yaml
# For 3D map server
# combined_params_3d.yaml
map_server_3d1:
    ros__parameters:
        yaml_filename: "some_map_3d.yaml"

map_server_3d2:
    ros__parameters:
        yaml_filename: "another_map_3d.yaml"
```

Then, one would invoke this process with the params file that contains the parameters for both nodes:

```shell
process_with_multiple_map_servers_2d __params:=combined_params_2d.yaml
```
```shell
process_with_multiple_map_servers_3d __params:=combined_params.yaml
```

#### Map Saver

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

- for an `image/point-cloud` as input
```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

## Currently Supported Map Types

- Occupancy grid (nav_msgs/msg/OccupancyGrid)
- PointCloud PointCloud2 (sensor_msgs/msg/PointCloud2)

## map_io_2d library

`map_io_2d` library contains following API functions declared in `map_2d/map_io_2d.hpp` to work with
OccupancyGrid maps:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the image from map file and generate an OccupancyGrid
- loadMapFromYaml(): Load the map YAML, image from map file and generate an OccupancyGrid
- saveMapToFile(): Write OccupancyGrid map to file

## map_io_3d library

`map_io_3d` library contains following API functions declared in `map_3d/map_io_3D.hpp` to work with
3D(PointCloud) maps:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the pcd from map file and generate an PointCloud
- loadMapFromYaml(): Load the map YAML, image from map file and generate an PointCloud
- saveMapToFile(): Write PointCloud map to file

## Services

### 2D Map Saver and server
As in ROS navigation, the `map_server_2d` node provides a "map" service to get the map. 
See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, `map_server_2d` also now provides a "load_map" service and `map_saver_2d` -
a "save_map" service. See nav2_msgs/srv/LoadMap.srv and nav2_msgs/srv/SaveMap.srv for details.

For using these services `map_server_2d`/`map_saver_2d` should be launched as a continuously running
`nav2::LifecycleNode` node. In addition to the CLI, `MapSaver<nav_msgs::msg::OccupancyGrid>` has a functionality of 
server
handling incoming services. To run `MapSaver<nav_msgs::msg::OccupancyGrid>` in a server mode
`nav2_map_server/launch/map_saver_server_2d.launch.py` launch-file could be used.

Service usage examples:

```shell
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

### 3D Map Saver and Server
The `map_server_3d` provides the new services to handle the requests for 3D point-cloud based functionalities. The 
following services presently implemented with `map_server_3d`.

- a get map service with the name:-`{node_name}/map` definition:-`nav2_msgs/srv/GetMap3D.srv`
- a load map service with the name:-`{node_name}/load_map` definition:-`nav2_msgs/srv/LoadMap3D.srv`   
- a publisher publishing the `PCD2` message with name:-`{node_name}/lode_map` definition:-`nav2_msgs/msg/PCD2.msg`

The `map_saver_3d` extends support for 3D point-cloud saving functionalities, with the following services.
- a service for saving 3D map with the name:-`{node_name}/save_map3D` definition:-`nav2_msgs/srv/SaveMap3D.srv`

For using these services `map_server_3d`/`map_saver_3d` should be launched as a continuously running
`nav2::LifecycleNode` node. In addition to the CLI, `MapSaver<sensor_msgs::msg::PointCloud2>` has a functionality of
server
handling incoming services. To run `MapSaver<sensor_msgs::msg::PointCloud2>` in a server mode
`nav2_map_server/launch/map_saver_server_3d.launch.py` launch-file could be used.

```shell
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap3D "{map_url: /ros/maps/map.yaml}"
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap3D "{map_topic: map, origin_topic: map_origin, map_url: my_map, as_binary: <true/false>}"
```
