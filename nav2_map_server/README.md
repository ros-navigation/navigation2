# Map Server

The `Map Server` provides maps to the rest of the Nav2 system using both topic and
service interfaces.

## Changes from ROS1 Navigation Map Server

While the Nav2 Map Server provides the same general function as the Nav1 Map Server, the new
code has some changes to accommodate ROS2 as well as some architectural improvements.
It was modified to support both OccupancyGrids and PointClouds, as well as be extensible to any kind of map and services with minimal changes made. 

In addition, there is now new "load_map/load_map_3d" and "save_map/save_map_3d" services which can be used to dynamically load and save a map.

### Architecture 

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety of map types, and thus some aspects of the original code have been refactored to support this new extensible framework.

#### Extensible Framework

The Nav2 Map Server, leverages *template metaprogramming* to enable handling of new map types via *template specializations*. This provides switching between different server types via specifying map type as template parameter. Thus all specializations of map server share the same `LifecycleNode` structure, while giving to user a complete  access to the design. This approach includes integration of a map I/O system which is responsible for basic operations with any type of map, and server parts for handling services, messages etc. with these maps. It allows to have an isolated per each map type implementations of the complete map handling system.

Demonstration of *template metaprogramming* and *specializations*:

```c++
template<class mapT>
class ExampleClass {
};

template<>
class ExampleClass<specialization_1> {
};

template<>
class ExampleClass<specialization_2> {
};
```


In order to get a new implementation for other map type with the new map server, one needs to define a specialization of the base server and saver with the required services/messages types, then
add the declaration of this specialization into `map_server.hpp` and `map_saver.hpp` interface headers.

#### Architecture in detail

Currently, map server divides into four parts:

 1. `Map Server`
 2. `Map Saver`
 3. `MapIO`
	 -  `MapIO 2D`
	 -  `MapIO 3D`
4. `Costmap Info Server`

`Map Server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`Map Saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`MapIO 2D` - is a map input-output library for `OccupancyGrids`. The library is designed to be an object-independent in order to allow easily save/load map from external code just by calling necessary function. This library is also used by `Map Server` and `Map Saver` to work. It supports OccupancyGrid saving/loading functions moved from the rest part of map server code. It is designed to be replaceable for a new IO library (e.g. for library with a new map encoding method or any other library supporting costmaps, multifloor maps, etc...).

`MapIO 3D` - is a 3D analogue of `MapIO 2D`: map input-output library for `PointClouds` and deals especially with `pcd` file format, described at [pcl website](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html#pcd-file-format).

**Note:** In the present implementation `MapServer` and `MapSaver` uses 2D and 3D maps reflecting their respective *specializations* as follows below.

For 2D case:
```c++
// map_2d/map_server_2d.hpp
// map_2d/map_server_2d.cpp
template<>
class MapServer<nav_msgs::msg::OccupancyGrid> : public nav2_util::LifecycleNode {...};

// map_2d/map_saver_2d.hpp
// map_2d/map_saver_2d.cpp
template<>
class MapSaver<nav_msgs::msg::OccupancyGrid> : public nav2_util::LifecycleNode {...};
```

For  3D case:
```c++
// map_3d/map_server_3d.hpp
// map_3d/map_server_3d.cpp
template<>
class MapServer<sensor_msgs::msg::PointCloud2> : public nav2_util::LifecycleNode {...};

// map_3d/map_saver_3d.hpp
// map_3d/map_saver_3d.cpp
template<>
class MapSaver<sensor_msgs::msg::PointCloud2> : public nav2_util::LifecycleNode {...};
```

#### Library interfaces list

The following list provides the headers mapped to their present functionalities. For example, if one wants to use  the server node then adding `#include "map_server/map_server.hpp"` will provide the server node implementation with a respective specialization of the selected map type.

| Basic Components | Functionality | Namespace | Contains
| :-----------------: | :-----------------: | :------------------: | :------------: |
| map_server.hpp | Template Map Server | nav2_map_server | `MapServer<OccupancyGrid>()`<br /> `MapServer<Pointcloud2>()`
| map_saver.hpp | Template Map Saver | nav2_map_server | `MapSaver<OccupancyGrid>()`<br /> `MapSaver<Pointcloud2>()`
| map_io_2d.hpp | 2D Map IO | nav2_map_server::map_2d | Functions for OccupancyGrid I/O operations
| map_io_3d.hpp | 3D Map IO | nav2_map_server::map_3d | Functions for PointCloud2 I/O operations

#### Existing Executables

The existing system of Nav2 Map Server provides many executables to get a hassle free services launch and handling. Listed below all the executables with their callable names, and functionalities:

| Executables | Implementation | Functionality | Type | Map Type
| :-------------: | :-----------------: | :--------------: | :----: | :--- |
| map_server_2d | main_2d.cpp | Map Server | LifecycleNode | OccupancyGrid
| map_server_3d | main_3d.cpp | Map Server | LifecycleNode | PointCloud2
| map_saver_server_2d | main_server_2d.cpp | Map Saver Server | LifecycleNode | OccupancyGrid
| map_saver_server_3d | main_server_3d.cpp | Map Saver Server | LifecycleNode | PointCloud2
| map_saver_cli | main_cli.cpp | Map Saver CLI | CLI | Detects based<br /> on input parameters

Executables of `LifecycleNode` type are being running along with lifecycle node manager e.g. through a launch file, while `map_saver_cli` is designed to be called directly through a command line.

### CLI-usage
  
#### Map Server 

The `Map Server` is a composable ROS2 node. By default, there is a `map_server_2d` and `map_server_3d` executable that instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1. With ROS1, one invoked the map server and passing the map YAML filename, like this:

```shell
$ map_server map_2d.yaml
```
Where the YAML file specified contained the various map metadata, such as:

```yaml
# map_2d.yaml
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Nav2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a level of indirection to get the map yaml filename. For example, for a node named 'map_server', the parameter file would look like this:

```yaml
# map_server_params_2d.yaml
map_server:
    ros__parameters:
        yaml_filename: "map_2d.yaml"
```

For 3D map server YAML consists of two main arguments file name as `image`, and view point as `origin` :
```yaml
image: testmap.pcd
origin: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
```
and the `parameters file` can be  defined as shown below:
 
```yaml
# map_server_params_3d.yaml
map_server:
    ros__parameters:
        yaml_filename: "map_3d.yaml"
```

Below is the process to use `parameters file` with ROS2:

```shell
# For 2D Map Server
$ ros2 run nav2_map_server map_server_2d --ros-args --params-file <map_server_params_2d.yaml>
# For 3D Map Server
$ ros2 run nav2_map_server map_server_3d --ros-args --params-file <map_server_params_3d.yaml>
```

**Note:** As any `LifecycleNode`, Map Server needs to be configured and activated via a call for the state change. This could be done for example by the following code snippet:

```c++
// map_client.cpp
// Change state of node to configure
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "nav2_util/lifecycle_service_client.hpp"

using lifecycle_msgs::msg::Transition;

static rclcpp::Node::SharedPtr node_ = 
  rclcpp::Node::make_shared("map_client");
  
// Create the lifecycle client
std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_ = 
  std::make_shared<nav2_util::LifecycleServiceClient>("map_server", node_);

std::this_thread::sleep_for(std::chrono::seconds(5));
const std::chrono::seconds timeout(5.0);

// Configure and activate the node to launch the services
lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, timeout);
lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, timeout);

// // // // // ///
// Do something //
// // // // // ///

// Done with work, shut down the node
lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);
lifecycle_client_->change_state(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
```

- `CMakeFile.txt` for the given above `map_client.cpp`:

```make - Makefile
cmake_minimum_required(VERSION 3.5)

project(map_server_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)

add_executable(map_client 
  map_client.cpp)

ament_target_dependencies(server_init
  rclcpp
  nav2_util)

install(TARGETS server_init
  RUNTIME DESTINATION lib/${PROJECT_NAME})
```

- Command to run generated executable in parallel with `map_server_2d/map_server_3d`

```shell
// In the workspace directory
// Use colcon build to get the package build and installed
// Use zsh or bash shell 
$ source <Workspace>/install/setup.<zsh/bash> 
$ ros2 run map_server_client server_init 
```

#### Map Saver

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

- The CLI is now able to work with both `OccupancyGrids` and `PointClouds`.
-  It detects the type of map to be saved, and utilizes appropriate `MapSaver*` node.
- To view the available parameters for `2D` and `3D` saver, please use ```-h``` flag to get help message.

#### Composible nodes

There is also possibility of having multiple map server/saver nodes of different types in a single process, where each node parameters would be separated by its name like presented below:

- Map server
```yaml
# For composite MapServer launch
# examples/multi_map_server_node/multi_composite_node_params.yaml
map_server_2d_n1:
    ros__parameters:
        yaml_filename: "./occupancy_grid_map.yaml"
map_server_2d_n2:
    ros__parameters:
        yaml_filename: "./occupancy_grid_map.yaml"
map_server_3d_n3:
    ros__parameters:
        yaml_filename: "./point_cloud_map.yaml"
map_server_3d_n4:
    ros__parameters:
        yaml_filename: "./point_cloud_map.yaml"
```

- Map saver
```yaml
# For composite MapSaver launch
# examples/multi_map_saver_node/multi_composite_node_params.yaml
map_saver_server_2d_1:
    ros__parameters:
        save_map_timeout: 2.0
        free_thresh_default: 0.25
        occupancy_thresh_default: 0.65
map_saver_server_2d_2:
    ros__parameters:
        save_map_timeout: 2.0
        free_thresh_default: 0.25
        occupancy_thresh_default: 0.5
map_saver_server_3d_3:
    ros__parameters:
        save_map_timeout: 5.0
map_saver_server_3d_4:
    ros__parameters:
        save_map_timeout: 4.0
```

Then, one can invoke the process via launch file, with the YAML-file that contains the parameters for both nodes:

```shell
# For Map Server
# Files are placed in the
# `nav2_map_server/examples/multi_map_saver_node` directory
$ ros2 launch multi_composite_map_saver.launch.py params_file:="multi_composite_node_params.yaml"
# For Map Saver
$ ros2 launch multi_composite_map_server.launch.py params_file:="multi_composite_node_params.yaml"
```

## Supported Map Types

- Occupancy grid (nav_msgs/msg/OccupancyGrid)
- PointCloud PointCloud2 (sensor_msgs/msg/PointCloud2)

For now, there are only two main map types are supported one for `2D`(OccupancyGrid) and other for `3D`(PointCloud), still one can implement their own with full freedom to hold all the aspects of the new map type. Such extensions could be simply supported in Map Server. For that it is required to define the template specialization of the core (bare minimum) implementation of the new map type for Map Server, Map Saver and Map IO library. Also the new map type can use either a preimplemented `MapIO 2D` and `MapIO 3D`, or a custom I/O library to handle map files, and message conversions.

## MapIO 2D library

`MapIO 2D` library contains following API functions declared in `map_2d/map_io_2d.hpp` to work with
`OccupancyGrid` maps:

- map_2d::loadMapYaml(): Load and parse the given YAML file
- map_2d::loadMapFromFile(): Load the image from map file and generate an `OccupancyGrid`
- map_2d::loadMapFromYaml(): Load the map YAML, image from map file and generate an `OccupancyGrid`
- map_2d::saveMapToFile(): Write `OccupancyGrid` map to file

This one provides `OccupancyGrid` operations functionality to `Map Server/Saver` template specialization for 2D image like maps.
 
## MapIO 3D library

`MapIO 3D` library contains following API functions declared in `map_3d/map_io_3d.hpp` to work with
`PointCloud` maps:

- map_3d::loadMapYaml(): Load and parse the given YAML file
- map_3d::loadMapFromFile(): Load the pcd from map file and generate an `PointCloud`
- map_3d::loadMapFromYaml(): Load the map YAML, image from map file and generate an `PointCloud`
- map_3d::saveMapToFile(): Write `PointCloud` map to file

This one provides `PointCloud` handling functionality to `Map Server/Saver` template specialization for 3D point cloud maps.

## Services

As in ROS navigation, the `map_server` node provides a "map" service to get the map. See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, `map_server` also now provides a "load_map" service and `map_saver` - a "save_map" service. See nav2_msgs/srv/LoadMap.srv and nav2_msgs/srv/SaveMap.srv for details.

NEW in ROS2 Galactic, the map_server also provides "get_map_3d", "load_map_3d", and "saver_map_3d" to encorporate the `PointCloud` maps. See nav2_msgs/srv/GetMap3D.srv, nav2_msgs/srv/LoadMap3D.srv and nav2_msgs/srv/SaveMap3D.srv for more details on this. 

Here listed the details for all the available services:

### Occupancy Grid services

1. 2D Map Server
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

### Point Cloud services
1. 3D Map Server
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
2. 3D Map Saver
- SaveMap
	- nav2_msgs/srv/SaveMap3D.srv
```yaml
# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_topic
string origin_topic
string map_url
# as_binary: If true then map will be saved as a binary file
# else ASCII format will be used.
bool as_binary
# Constants for image_format. Supported formats: pcd, ply
string file_format
---
bool result
```

---
### Service Usage
 
For using these services `map_server/map_saver` should be launched as a continuously running `nav2::LifecycleNode` node. In addition to the CLI, Map Saver has a functionality of server handling incoming services. To run Map Saver in a server mode, provided launch-files could be used.
 
| LaunchFile | Assiciated executable | Functionality | Type | Map Type
| :-------------: | :-----------------: | :--------------: | :----: | :---: |
| map_saver_server_2d.launch.py | main_server_2d.cpp | Map Saver | LifcyleNode | OccupancyGrid
| map_saver_server_3d.launch.py | main_server_3d.cpp |  Map Saver | LifcyleNode | PointCloud2

```shell
$ ros2 launch nav2_map_server map_saver_server_2d.launch.py
```
```shell
$ ros2 launch nav2_map_server map_saver_server_3d.launch.py
```

**Note:** Although there is no launch file directly executing `map_server`, its launching principles are the same as for `map_saver_server` and could be found in `nav2_bringup/bringup/launch/localization_launch.py` file.

Service usage examples:
- 2D Map Server and Saver
```shell
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```
- 3D Map Server and Saver
```shell
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap3D "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap3D "{map_topic: map, origin_topic: map_origin, map_url: my_map, as_binary: <true/false>}"
```
