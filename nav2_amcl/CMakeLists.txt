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

include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-folding-constant)
endif()

add_library(pf_lib SHARED
  src/pf/pf.c
  src/pf/pf_kdtree.c
  src/pf/pf_pdf.c
  src/pf/pf_vector.c
  src/pf/eig3.c
  src/pf/pf_draw.c
)
target_include_directories(pf_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
if(HAVE_DRAND48)
  target_compile_definitions(pf_lib PRIVATE "HAVE_DRAND48")
endif()

add_library(map_lib SHARED
  src/map/map.c
  src/map/map_range.c
  src/map/map_draw.c
  src/map/map_cspace.cpp
)
target_include_directories(map_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_library(motions_lib SHARED
  src/motion_model/omni_motion_model.cpp
  src/motion_model/differential_motion_model.cpp
)
target_include_directories(motions_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(motions_lib pf_lib pluginlib::pluginlib nav2_util::nav2_util_core)

add_library(sensors_lib SHARED
  src/sensors/laser/laser.cpp
  src/sensors/laser/beam_model.cpp
  src/sensors/laser/likelihood_field_model.cpp
  src/sensors/laser/likelihood_field_model_prob.cpp
)
target_include_directories(sensors_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(sensors_lib pf_lib map_lib)

set(executable_name amcl)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/amcl_node.cpp
)
if(HAVE_DRAND48)
  target_compile_definitions(${library_name} PRIVATE "HAVE_DRAND48")
endif()
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  nav2_util::nav2_util_core
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  pluginlib::pluginlib
  sensors_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
  ${nav2_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
  ${tf2_geometry_msgs_TARGETS}
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${executable_name} PRIVATE
  ${library_name}
)

rclcpp_components_register_nodes(${library_name} "nav2_amcl::AmclNode")

install(TARGETS ${library_name} pf_lib map_lib motions_lib sensors_lib
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
ament_export_dependencies(geometry_msgs message_filters nav2_util sensor_msgs std_srvs pluginlib rclcpp rclcpp_lifecycle nav_msgs tf2_ros tf2 nav2_msgs)
ament_export_targets(${library_name})
pluginlib_export_plugin_description_file(nav2_amcl plugins.xml)
ament_package()
