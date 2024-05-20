cmake_minimum_required(VERSION 3.5)
project(nav2_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)

nav2_package()

set(executable_name bt_navigator)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
)
target_include_directories(${library_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  nav2_util::nav2_util_core
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
  nav2_core::nav2_core
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_link_libraries(${library_name} PRIVATE
  nav2_behavior_tree::nav2_behavior_tree
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

add_library(nav2_navigate_to_pose_navigator SHARED src/navigators/navigate_to_pose.cpp)
target_include_directories(nav2_navigate_to_pose_navigator
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_navigate_to_pose_navigator PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
)
target_link_libraries(nav2_navigate_to_pose_navigator PRIVATE
  rclcpp_lifecycle::rclcpp_lifecycle
  ament_index_cpp::ament_index_cpp
  nav2_behavior_tree::nav2_behavior_tree
  pluginlib::pluginlib
)

add_library(nav2_navigate_through_poses SHARED src/navigators/navigate_through_poses.cpp)
target_include_directories(nav2_navigate_through_poses
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_navigate_through_poses PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
)
target_link_libraries(nav2_navigate_through_poses PRIVATE
  rclcpp_lifecycle::rclcpp_lifecycle
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
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

ament_export_include_directories(include)
ament_export_libraries(${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs nav2_core nav_msgs nav2_msgs nav2_util tf2_ros pluginlib)
ament_export_targets(export_${library_name})
ament_package()
