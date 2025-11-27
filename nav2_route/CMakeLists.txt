cmake_minimum_required(VERSION 3.8)
project(nav2_route CXX)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nanoflann REQUIRED)
find_package(nlohmann_json REQUIRED)

nav2_package()

set(executable_name route_server)
set(library_name ${executable_name}_core)

include_directories(
  include
)

set(dependencies
  nav2_core
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rcl_interfaces
  geometry_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  tf2_ros
  tf2
  angles
  nlohmann_json
)

# Main library
add_library(${library_name} SHARED
  src/route_server.cpp
  src/route_planner.cpp
  src/route_tracker.cpp
  src/edge_scorer.cpp
  src/operations_manager.cpp
  src/node_spatial_tree.cpp
  src/path_converter.cpp
  src/graph_loader.cpp
  src/graph_saver.cpp
  src/goal_intent_extractor.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} PRIVATE ${library_name})

rclcpp_components_register_nodes(${library_name} "nav2_route::RouteServer")

# Edge scoring plugins
add_library(edge_scorers SHARED
  src/plugins/edge_cost_functions/distance_scorer.cpp
  src/plugins/edge_cost_functions/time_scorer.cpp
  src/plugins/edge_cost_functions/dynamic_edges_scorer.cpp
  src/plugins/edge_cost_functions/penalty_scorer.cpp
  src/plugins/edge_cost_functions/costmap_scorer.cpp
  src/plugins/edge_cost_functions/semantic_scorer.cpp
  src/plugins/edge_cost_functions/goal_orientation_scorer.cpp
  src/plugins/edge_cost_functions/start_pose_orientation_scorer.cpp
)
ament_target_dependencies(edge_scorers
  ${dependencies}
)

# Route operations plugins
add_library(route_operations SHARED
  src/plugins/route_operations/adjust_speed_limit.cpp
  src/plugins/route_operations/trigger_event.cpp
  src/plugins/route_operations/rerouting_service.cpp
  src/plugins/route_operations/collision_monitor.cpp
  src/plugins/route_operations/time_marker.cpp
)
ament_target_dependencies(route_operations
  ${dependencies}
)

# Graph Parser plugins
add_library(graph_file_loaders SHARED
    src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp
)
ament_target_dependencies(graph_file_loaders
  ${dependencies}
)

add_library(graph_file_savers SHARED
    src/plugins/graph_file_savers/geojson_graph_file_saver.cpp
)
ament_target_dependencies(graph_file_savers
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_route plugins.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name} edge_scorers route_operations graph_file_loaders graph_file_savers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY graphs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/test_graphs DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name} edge_scorers route_operations graph_file_loaders graph_file_savers)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
