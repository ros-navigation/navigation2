cmake_minimum_required(VERSION 3.5)
project(nav2_smac_planner)

set(CMAKE_BUILD_TYPE Release) #Debug, Release

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ompl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

set(library_name nav2_smac_planner)

# Common library
add_library(${library_name}_common SHARED
  src/a_star.cpp
  src/analytic_expansion.cpp
  src/collision_checker.cpp
  src/costmap_downsampler.cpp
  src/node_2d.cpp
  src/node_basic.cpp
  src/node_hybrid.cpp
  src/node_lattice.cpp
  src/smoother.cpp
)
target_include_directories(${library_name}_common
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${library_name}_common PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name}_common PRIVATE
  angles::angles
)

# Hybrid plugin
add_library(${library_name} SHARED
  src/smac_planner_hybrid.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${library_name} PUBLIC
  ${library_name}_common
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  pluginlib::pluginlib
)

# 2D plugin
add_library(${library_name}_2d SHARED
  src/smac_planner_2d.cpp
)
target_include_directories(${library_name}_2d
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${library_name}_2d PUBLIC
  ${library_name}_common
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name}_2d PRIVATE
  pluginlib::pluginlib
)

# Lattice plugin
add_library(${library_name}_lattice SHARED
  src/smac_planner_lattice.cpp
)
target_include_directories(${library_name}_lattice
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${library_name}_lattice PUBLIC
  ${library_name}_common
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name}_lattice PRIVATE
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nav2_core smac_plugin_hybrid.xml)
pluginlib_export_plugin_description_file(nav2_core smac_plugin_2d.xml)
pluginlib_export_plugin_description_file(nav2_core smac_plugin_lattice.xml)

install(TARGETS ${library_name}_common ${library_name} ${library_name}_2d ${library_name}_lattice
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY lattice_primitives/sample_primitives DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE include/nav2_smac_planner/thirdparty/robin_hood.h)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} ${library_name}_2d ${library_name}_lattice)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  nlohmann_json
  ompl
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2
  tf2_ros
  visualization_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()
