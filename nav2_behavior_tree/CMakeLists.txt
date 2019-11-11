cmake_minimum_required(VERSION 3.5)
project(nav2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# The behavior tree (bt) node plugins
set(bt_plugins
  compute_path_to_pose_action
  follow_path_action
  back_up_action
  spin_action
  wait_action
  clear_costmap_service
  is_stuck_condition
  goal_reached_condition
  initial_pose_received_condition
  reinitialize_global_localization_service
  rate_controller
  recovery_node
)

foreach(bt_plugin ${bt_plugins})

  set(lib_name "nav2_${bt_plugin}_bt_node")

  add_library(${lib_name} SHARED plugins/${bt_plugin}.cpp)

  ament_target_dependencies(${lib_name} ${dependencies})

  target_compile_definitions(${lib_name} PRIVATE BT_PLUGIN_EXPORT)

  list(APPEND plugin_libs ${lib_name})

endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
