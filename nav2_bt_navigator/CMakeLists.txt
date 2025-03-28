cmake_minimum_required(VERSION 3.5)
project(nav2_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name bt_navigator)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${library_name}
  nav2_core
  nav2_util
  pluginlib
  rclcpp
  rclcpp_action
  tf2_ros
  nav2_behavior_tree
  rclcpp_components
  rclcpp_lifecycle
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${executable_name}
  rclcpp
)
target_link_libraries(${executable_name}
  ${library_name}
)

add_library(nav2_navigate_to_pose_navigator SHARED src/navigators/navigate_to_pose.cpp)
target_include_directories(nav2_navigate_to_pose_navigator
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(nav2_navigate_to_pose_navigator
  nav2_behavior_tree
  geometry_msgs
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  ament_index_cpp
  pluginlib
  rclcpp_lifecycle
)

add_library(nav2_navigate_through_poses SHARED src/navigators/navigate_through_poses.cpp)
target_include_directories(nav2_navigate_through_poses
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(nav2_navigate_through_poses
  geometry_msgs
  nav2_core
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  ament_index_cpp
  pluginlib
  rclcpp_lifecycle
  nav2_behavior_tree
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)
rclcpp_components_register_nodes(${library_name} "nav2_bt_navigator::BtNavigator")

install(TARGETS ${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses
  EXPORT export_${library_name}
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

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses)
ament_export_dependencies(geometry_msgs nav2_core nav2_msgs nav2_util nav_msgs pluginlib rclcpp rclcpp_action tf2_ros nav2_behavior_tree)
ament_export_targets(export_${library_name})
ament_package()
