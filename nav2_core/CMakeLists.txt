cmake_minimum_required(VERSION 3.5)
project(nav2_core)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
