cmake_minimum_required(VERSION 3.5)
project(nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name controller_server)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/controller_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  pluginlib::pluginlib
  nav2_util::nav2_util_core
  nav_2d_utils::conversions
  nav_2d_utils::tf_help
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${nav_2d_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

add_library(simple_progress_checker SHARED plugins/simple_progress_checker.cpp)
target_include_directories(simple_progress_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_progress_checker PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(simple_progress_checker PRIVATE
  nav2_util::nav2_util_core
  nav_2d_utils::conversions
  pluginlib::pluginlib
)

add_library(pose_progress_checker SHARED plugins/pose_progress_checker.cpp)
target_include_directories(pose_progress_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pose_progress_checker PUBLIC
  simple_progress_checker
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(pose_progress_checker PRIVATE
  pluginlib::pluginlib
  nav2_util::nav2_util_core
  nav_2d_utils::conversions
  angles::angles
)

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
target_include_directories(simple_goal_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_goal_checker PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_core::nav2_core
  ${rcl_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
)
target_link_libraries(simple_goal_checker PRIVATE
  pluginlib::pluginlib
  angles::angles
  nav2_util::nav2_util_core
  tf2::tf2
)

add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
target_include_directories(stopped_goal_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(stopped_goal_checker PUBLIC
  simple_goal_checker
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(stopped_goal_checker PRIVATE
  pluginlib::pluginlib
  nav2_util::nav2_util_core
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()

  add_subdirectory(test)
  add_subdirectory(plugins/test)
endif()

rclcpp_components_register_nodes(${library_name} "nav2_controller::ControllerServer")

install(TARGETS simple_progress_checker pose_progress_checker simple_goal_checker stopped_goal_checker ${library_name}
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

ament_export_include_directories(include)
ament_export_libraries(simple_progress_checker
  pose_progress_checker
  simple_goal_checker
  stopped_goal_checker
  ${library_name})
ament_export_dependencies(nav2_core nav2_costmap_2d tf2_ros nav2_msgs pluginlib nav2_util nav_2d_utils rclcpp rcl_interfaces rclcpp_lifecycle geometry_msgs nav_2d_msgs)
ament_export_targets(${library_name})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
