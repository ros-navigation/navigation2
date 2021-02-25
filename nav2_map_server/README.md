# Nav2 Map Server

The `Map Server` provides maps to the rest of the Nav2 system using both topic and
service interfaces.

## Changes from ROS1 Navigation Map Server

The new nav2 map server is modified to support both OccupancyGrids and PointCLouds.Also able to be extended to any map type and services with minimal changes required.   

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework.

#### Extensible Framwork :cool:

The new Nav2 Map Server, leverages `template metaprogramming` to enable handling of new map-types via `tamplate specializations`. This provides switching b/w different server types via. specifying map-type as template parameter.  Thus all of the map-servers shares same `LifecycleNode` structure, while giving user complete access to design and integrate new I/O system for any type of map, and the handling utility `services`, `messages` etc. and most importantly, allows the implementation isolation of the complete system for one map-type  from another. 

In order to get a new implementation for other map-type with the new map-server, one needs to define a specialization of the base server and saver with the required services/messages types, and add the declaration of specialization in the interface header `map_server.hpp` and `map_saver.hpp`.

Currently, map server divides into four parts:

 1. `Map Server`
 2. `Map Saver`
 3. `MapIO`
	 -  `MapIO 2D`
	 -  `MapIO 3D`
4. Costmap Info Server

##
`Map Server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`Map Saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`MapIO 2D` - is a map input-output library for `occupancy-grids`. The library is designed to be an object-independent in order to allow easily save/load map from external code just by calling necessary function. This library is also used by `Map Server` and `Map Saver`. It supports OccupancyGrid saving/loading functions moved from the rest part of map server code. It is designed to be replaceable for a new IO library (e.g. for library with a new map encoding method or any other library supporting costmaps, multifloor maps, etc...).

`MapIO 3D` - is a map input-output library for `PointClouds` and deals especially with `.pcd` 
file I/O for more details about `pcd` file format please refer to 
[pcl website](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html#pcd-file-format). The library is used by `Map Server` and `Map Saver`.

### :cyclone:Library interfaces list :point_down:
| Implementations<br />nav2_map_server/ | Functionality | namespace | Usage
| :-----------------: | :-----------------: | :------------------: | :------------: |
| map_server.hpp | Template Map Server | nav2_map_server | `MapServer<OccupancyGrid>()`<br /> `MapServer<Pointcloud2>()`
| map_saver.hpp | Template Map Saver | nav2_map_server | `MapSaver<OccupancyGrid>()`<br /> `MapSaver<Pointcloud2>()`
| map_io_2d.hpp | 2D Map IO | nav2_map_server::map_2d | Functions for OccupancyGrid handling
| map_io_3d.hpp | 3D Map IO | nav2_map_server::map_3d | Functions for PointCloud2 handling

##

### CLI-usage :tiger2:

The exissing system of Nav2 Map Server provides many executable to get a hasle free services launch and handling. Detailed list and functionalities are given :point_down: 

:point_right: Executable list:
| Executables | implementation | Functionality | Type | Map Type
| :-------------: | :-----------------: | :--------------: | :----: | :--- |
| map_server_2d | main_2d.cpp | Map Server | LifcyleNode | OccupancyGrid
| map_server_3d | main_3d.cpp | Map Server | LifcyleNode | PointCloud2
| map_saver_server_2d | main_server_2d.cpp | Map Saver | LifcyleNode | OccupancyGrid
| map_saver_server_3d | main_server_3d.cpp | Map Saver | LifcyleNode | PointCloud2
| map_saver_cli | main_cli.cpp | Map Saver CLI | CLI | Detects based<br /> on Parameters
  
#### Map Server :tiger2:

The `Map Server` is a composable ROS2 node. By default, there is a `map_server_2d` and `map_server_3d` executable that instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

:point_right: 2D Server
```shell
$ map_server_2d map_2d.yaml
```

Where the YAML file specified contained the various map metadata, such as:
```yaml
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

:point_right: 3D Server
```
$ map_server_3d map_3d.yaml
```

For 3D map server YAML consists of two main arguments file name as `image`, and view point as `origin` :

```yaml
image: testmap.pcd
origin: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
```

:point_right: Parameters files

The Navigation2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a level of indirection to get the map yaml filename. For example, for a node named 'map_server', the parameter file would look like this:

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
like :point_down:

```shell
# For 2D server
map_server_2d __params:=map_server_params_2d.yaml
```
```shell
# For 3D server
map_server_3d __params:=map_server_params_3d.yaml
```

If needed there services can be launched by using the provided launch files. 
Lunch Scripts :point_down:
| LaunchFile | Assiciated executable | Functionality | Type | Map Type
| :-------------: | :-----------------: | :--------------: | :----: | :---: |
| map_saver_server_2d.launch.py | main_server_2d.cpp | Map Saver | LifcyleNode | OccupancyGrid
| map_saver_server_3d.launch.py | main_server_3d.cpp |  Map Saver | LifcyleNode | PointCloud2

```shell
$ ros2 launch map_saver_server_2d.launch.py"
```
```shell
$ ros2 launch map_saver_server_3d.launch.py "
```

:point_right: Launch multiple nodes using launch file with parameters through YAML
There is also possibility of having multiple map server nodes in a single process, where the parameters file would separate the parameters by node name, like this:

```yaml
# For 2D map server
# combined_params_2d.yaml
map_server_2d_1:
    ros__parameters:
        yaml_filename: "some_map_2d.yaml"

map_server_2d_2:
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

Then, one can invoke the process via. launch file, with the params file that contains the parameters for both nodes:

```shell
$ ros2 launch <launch_file_name> params_file:="combined_params_2d.yaml"
```
```shell
$ ros2 launch <launch_file_name> params_file:="combined_params_3d.yaml"
```

:cyclone: :snowflake: :cyclone: :snowflake: :cyclone: :snowflake:

**Note:-** *Use the params_file as the input to parameters to a node launch command. Please refer to `/example/` folder for related demonstrations.*

:cyclone: :snowflake: :cyclone: :snowflake: :cyclone: :snowflake:


#### Map Saver :tiger2:

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

- With the `new map server`,  the CLI is now able to work with both `Occupancy Grid` and `Point Clouds`.
- The CLI internally detects the type of `Saver` needed, and exeutes related `Saver` utility.
- You can view the available parameters for `2D` and `3D` saver with ```-h``` flag to the run command.
- The run command is given here :point_down:
```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

## Supported Map Types :tiger2:

- Occupancy grid (nav_msgs/msg/OccupancyGrid)
- PointCloud PointCloud2 (sensor_msgs/msg/PointCloud2)

Presently, only two main map-types are supported one for `2D` and other for `3D`, still one can implement their own with full freedom to hold all the aspects of your new `map type`. 
> one needs to implement or use an existing I/O system.

> Create a specialization for your map type e.g :point_down:
```c++
template<>MapServer<new_mapT> : public nav2_util::LifecycleNode{};
```

>  if a new type of service/message is required one can write and directly add the service with a callback in the `specialization`.

For more information on adding a new map-type, simply analyse 
```c++ 
// 3D map_server
include/nav2_map_server/map_server_core.hpp
include/nav2_map_server/map_3d/map_server_3d.hpp
src/map_3d/map_server_3d.cpp
include/nav2_map_server/map_server.hpp

// 3D map_saver
include/nav2_map_server/map_saver_core.hpp
include/nav2_map_server/map_3d/map_saver_3d.hpp
src/map_3d/map_saver_3d.cpp
include/nav2_map_server/map_saver.hpp
``` 

## MapIO 2d library :tiger2:

`MapIO 2D` library contains following API functions declared in `map_2d/map_io_2d.hpp` to work with
OccupancyGrid maps:

- map_2d::loadMapYaml(): Load and parse the given YAML file
- map_2d::loadMapFromFile(): Load the image from map file and generate an OccupancyGrid
- map_2d::loadMapFromYaml(): Load the map YAML, image from map file and generate an OccupancyGrid
- map_2d::saveMapToFile(): Write OccupancyGrid map to file

This one provides `Occupancy Grid` handling functionality to `Map Server/Saver` template specialization for 2D image like maps.
 
## MapIO 3D library :tiger2:

`MapIO 3D` library contains following API functions declared in `map_3d/map_io_3d.hpp` to work with
PointCloud maps:

- map_3d::loadMapYaml(): Load and parse the given YAML file
- map_3d::loadMapFromFile(): Load the pcd from map file and generate an PointCloud
- map_3d::loadMapFromYaml(): Load the map YAML, image from map file and generate an PointCloud
- map_3d::saveMapToFile(): Write PointCloud map to file

This one provides `Point Cloud` handling functionality to `Map Server/Saver` template specialization for 3D point cloud like maps.

## Services :tiger2:

### Map Server and Saver :tiger2:

1. 2D Map server
- GerMap 
	- nav_msgs/srv/GetMap.srv
```yaml
# Get the map as a nav_msgs/OccupancyGrid
---
nav_msgs/OccupancyGrid map
```

- LoadMap
	- nav2_msgs/srv/LoadMap.srv
```yaml
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255
# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result
```

2. 2D Map Saver
- SaveMap
	- nav2_msgs/srv/SaveMap.srv
```yaml
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_topic
string map_url
# Constants for image_format. Supported formats: pgm, png, bmp
string image_format
# Map modes: trinary, scale or raw
string map_mode
# Thresholds. Values in range of [0.0 .. 1.0]
float32 free_thresh
float32 occupied_thresh
---
bool result
```

3. 3D Map server
- GetMap
	- nav2_msgs/srv/GetMap3D.srv
```yaml
# Get the map as a sensor_msgs/PointCloud2
---
# The current map hosted by this map service.
sensor_msgs/PointCloud2 map
geometry_msgs/Pose origin
```

- LoadMap
	- nav2_msgs/srv/LoadMap3D.srv
```yaml
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url
---
# Result code definitions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255
# Returned map is only valid if result equals RESULT_SUCCESS
sensor_msgs/PointCloud2 map
geometry_msgs/Pose origin
uint8 result
```
4. 3D Map saver
- SaveMap
	- nav2_msgs/srv/SaveMap3D.srv
```yaml
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_topic
string origin_topic
string map_url
# as_binary:- If true then map will be saved as a binary file
# else ASCII format will be used.
bool as_binary
# Constants for image_format. Supported formats: pcd, ply
string file_format
---
bool result
```

### Using Map Saver and Server :tiger2:

#### Using service call :tiger2:
For using these services via service call, we need to aunch the required the continuously running `Lifecycle Node`, either with the given executables:- `map_server_2d`/`map_saver_2d`/`map_server_3d`/`map_saver_3d`, or using launch files.
 Once done any program can call the running services (use `ros2 node list` to get the list of nodes and `ros2 service list` to get the running services.)

Service usage examples:
- 2D Map Server and Saver
```shell
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```
- 3D Map Server and Saver
```shell
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap3D "{map_url: /ros/maps/map.yaml}"
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap3D "{map_topic: map, origin_topic: map_origin, map_url: my_map, as_binary: <true/false>}"
```

