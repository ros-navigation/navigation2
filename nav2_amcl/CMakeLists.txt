cmake_minimum_required(VERSION 3.5)
project(nav2_amcl)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

add_subdirectory(src/pf)
add_subdirectory(src/map)
add_subdirectory(src/motion_model)
add_subdirectory(src/sensors)

set(executable_name amcl)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/amcl_node.cpp
)

target_include_directories(${library_name} PRIVATE src/include)

if(HAVE_DRAND48)
  target_compile_definitions(${library_name} PRIVATE "HAVE_DRAND48")
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  message_filters
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
  nav2_msgs
  pluginlib
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  map_lib pf_lib sensors_lib
)

rclcpp_components_register_nodes(${library_name} "nav2_amcl::AmclNode")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} pf_lib sensors_lib motions_lib map_lib)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_amcl plugins.xml)
ament_package()
