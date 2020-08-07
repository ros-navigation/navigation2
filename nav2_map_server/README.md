# Map Server

The `Map Server` provides maps to the rest of the Navigation2 system using both topic and
service interfaces.

## Changes from ROS1 Navigation Map Server

While the nav2 map server provides the same general function as the nav1 map server, the new
code has some changes to accommodate ROS2 as well as some architectural improvements.

In addition, there is now two new "load_map" and "save_map" services which can be used to
dynamically load and save a map.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework.

Currently, map server divides into tree parts:

- `map_server`
- `map_saver`
- `map_io` library
- `map_io_3d` library

`map_server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`map_saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`map_io` - is a map input-output library. The library is designed to be an object-independent
in order to allow easily save/load map from external code just by calling necessary function.
This library is also used by `map_loader` and `map_saver` to work. Currently, it contains
OccupancyGrid saving/loading functions moved from the rest part of map server code.
It is designed to be replaceable for a new IO library (e.g. for library with a new map encoding
method or any other library supporting costmaps, multifloor maps, etc...).

`map_io_3D` - is a counterpart of `map_io` library and deals with 3D data especially `.pcd` 
file I/O for more details about `pcd` file format please refer to 
[pcl website](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html#pcd-file-format).
 
### CLI-usage

#### Map Server

The `Map Server` is a composable ROS2 node. By default, there is a `map_server` executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1.
With ROS1, one invoked the map server and passing the map YAML filename, like this:

```
$ map_server map.yaml
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

- for a `pcd` as input/output 
```
image: testmap.pcd
origin: [0.0, 0.0, 0.0]
orientation: [1.0, 0.0, 0.0, 0.0]
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

#### Map Saver

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

- for an image as input
```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

- for a pcd as input
```
$ ros2 run nav2_map_server map_saver_cli_3D [arguments] [--ros-args ROS remapping args]
```

## Currently Supported Map Types

- Occupancy grid (nav_msgs/msg/OccupancyGrid)
- PointCloud PointCloud2 (sensor_msgs/msg/PointCloud2)

## MapIO library

`MapIO` library contains following API functions declared in `map_io.hpp` to work with
OccupancyGrid maps:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the image from map file and generate an OccupancyGrid
- loadMapFromYaml(): Load the map YAML, image from map file and generate an OccupancyGrid
- saveMapToFile(): Write OccupancyGrid map to file

## MapIO3D library

`MapIO3D` library contains following API functions declared in `map_io_3D.hpp` to work with
3D(PointCloud) maps, encapsulated in namespace `nav2_map_server_3d`:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the pcd from map file and generate an PointCloud
- loadMapFromYaml(): Load the map YAML, image from map file and generate an PointCloud
- saveMapToFile(): Write PointCloud map to file

## Services

As in ROS navigation, the `map_server` node provides a "map" service to get the map. 
See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, `map_server` also now provides a "load_map" service and `map_saver` -
a "save_map" service. See nav2_msgs/srv/LoadMap.srv and nav2_msgs/srv/SaveMap.srv for details.

For using these services `map_server`/`map_saver` should be launched as a continuously running
`nav2::LifecycleNode` node. In addition to the CLI, `Map Saver` has a functionality of server
handling incoming services. To run `Map Saver` in a server mode
`nav2_map_server/launch/map_saver_server.launch.py` launch-file could be used.

Service usage examples:

```
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```
### 3D support
The `map_server` is augmented to support 3D data, if a file with `.pcd` extension is given 
while configuring, the 3D supporting services will be activated with same service name as for 2D.     
Following will be enabled if file specified to read is with `.pcd` extension:
- a get map service with the name:-`{node_name}/map` definition:-`nav2_msgs/srv/GetMap3D.srv`
- a load map service with the name:-`{node_name}/load_map` definition:-`nav2_msgs/srv/LoadMap3D.srv`   
- a publisher publishing the `PCD2` message with name:-`{node_name}/lode_map` definition:-`nav2_msgs/msg/PCD2.msg`

The `map_saver` is also augmented to support 3D simultaneously with existing 2D capability.<br/> 
Following will be enabled on the general config:
- a service for saving 2D map with the name:-`{node_name}/save_map` definition:-`nav2_msgs/srv/SaveMap.srv`
- a service for saving 3D map with the name:-`{node_name}/save_map3D` definition:-`nav2_msgs/srv/SaveMap3D.srv`

*Note:- To run `Map Saver` in a server mode you can still use
`nav2_map_server/launch/map_saver_server.launch.py` launch-file could be used.*
       
###caution:-
__*map_server once configured can't be changed to different type of services e.g a server configured for 
images can't be changes to use pcd or the other way*__

Note:- `one can still change the parameters remaining in the limits of parameters 
defined for that type(image/pcd)` 
