cmake_minimum_required(VERSION 3.5)
project(costmap_2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

add_library(costmap_2d
  src/mock_costmap_2d.cpp
)

ament_target_dependencies(costmap_2d
  geometry_msgs
  tf2_ros
  tf2
)

install(TARGETS costmap_2d
       ARCHIVE DESTINATION lib
       LIBRARY DESTINATION lib
       )

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(costmap_2d)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2)
ament_package()
