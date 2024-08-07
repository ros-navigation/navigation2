cmake_minimum_required(VERSION 3.5)
project(nav2_route CXX)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rviz2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nanoflann REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

nav2_package()

set(executable_name route_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  visualization_msgs
  nav_msgs
  tf2_ros
  nav2_core
  nanoflann
  nlohmann_json
  rviz_common
  Qt5Core
  Qt5Gui
  Qt5Widgets
)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  include
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

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav2_route::RouteServer")

# Edge scoring plugins
add_library(edge_scorers SHARED
  src/plugins/edge_cost_functions/distance_scorer.cpp
  src/plugins/edge_cost_functions/time_scorer.cpp
  src/plugins/edge_cost_functions/adjust_edges_scorer.cpp
  src/plugins/edge_cost_functions/penalty_scorer.cpp
  src/plugins/edge_cost_functions/costmap_scorer.cpp
  src/plugins/edge_cost_functions/semantic_scorer.cpp
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

# Generate header for ui
qt5_wrap_ui(route_tool_UIS_H resource/route_tool.ui)
qt5_wrap_cpp(route_tool_MOCS include/nav2_route/route_tool/route_tool.hpp)

add_library(route_tool_rviz_panel SHARED
  src/route_tool/route_tool.cpp
  ${route_tool_UIS_H}
  ${route_tool_MOCS}
)

ament_target_dependencies(route_tool_rviz_panel ${dependencies})
target_link_libraries(route_tool_rviz_panel ${library_name} rviz_common::rviz_common)
target_compile_definitions(route_tool_rviz_panel PRIVATE QT_NO_KEYWORDS)

pluginlib_export_plugin_description_file(route_tool plugins.xml)
pluginlib_export_plugin_description_file(rviz_common rviz2_plugins.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name} edge_scorers route_operations graph_file_loaders graph_file_savers route_tool_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY graphs rviz launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name} edge_scorers route_operations graph_file_loaders graph_file_savers route_tool_rviz_panel)
ament_package()
