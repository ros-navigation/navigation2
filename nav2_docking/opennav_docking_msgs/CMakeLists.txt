cmake_minimum_required(VERSION 3.5)
project(opennav_docking_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DockRobot.action"
  "action/UndockRobot.action"
  "srv/ReloadDatabase.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
