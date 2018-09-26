cmake_minimum_required(VERSION 3.5)
project(nav2_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(costmap_lib SHARED
  src/costmap.cpp
)

ament_target_dependencies(costmap_lib
  rclcpp
  nav2_msgs
  tf2)

add_library(map_lib SHARED
  src/map/map.c
  src/map/map_store.c
  src/map/map_range.c
  src/map/map_draw.c
  src/map/map_cspace.cpp
)

add_library(pf_lib SHARED
  src/pf/pf.c
  src/pf/pf_kdtree.c
  src/pf/pf_pdf.c
  src/pf/pf_vector.c
  src/pf/eig3.c
  src/pf/pf_draw.c
)

add_library(sensors_lib SHARED
  src/sensors/amcl_sensor.cpp
  src/sensors/amcl_odom.cpp
  src/sensors/amcl_laser.cpp
)

install(TARGETS
  costmap_lib
  map_lib
  pf_lib
  sensors_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(costmap_lib pf_lib sensors_lib map_lib)

ament_package()
