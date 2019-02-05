cmake_minimum_required(VERSION 3.5)
project(nav2_msgs)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

nav2_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Costmap.msg"
  "msg/CostmapMetaData.msg"
  "msg/Path.msg"
  "msg/MissionPlan.msg"
  "msg/TaskStatus.msg"
  "msg/VoxelGrid.msg"
  "srv/GetCostmap.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
