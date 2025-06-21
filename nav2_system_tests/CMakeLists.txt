cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

nav2_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(angles REQUIRED)
  find_package(behaviortree_cpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav2_amcl REQUIRED)
  find_package(nav2_behavior_tree REQUIRED)
  find_package(nav2_lifecycle_manager REQUIRED)
  find_package(nav2_map_server REQUIRED)
  find_package(nav2_minimal_tb3_sim REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(nav2_navfn_planner REQUIRED)
  find_package(nav2_planner REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(navigation2 REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(nav2_ros_common REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  set(local_controller_plugin_lib local_controller)

  add_library(${local_controller_plugin_lib} SHARED src/error_codes/controller/controller_error_plugins.cpp)
  target_link_libraries(${local_controller_plugin_lib} PRIVATE
    ${geometry_msgs_TARGETS}
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    ${nav_msgs_TARGETS}
    pluginlib::pluginlib
  )
  pluginlib_export_plugin_description_file(nav2_core src/error_codes/controller_plugins.xml)
  target_include_directories(${local_controller_plugin_lib}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
      "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
  )

  install(TARGETS ${local_controller_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES src/error_codes/controller_plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )

  set(global_planner_plugin_lib global_planner)

  add_library(${global_planner_plugin_lib} SHARED src/error_codes/planner/planner_error_plugin.cpp)
  target_link_libraries(${global_planner_plugin_lib} PRIVATE
    ${geometry_msgs_TARGETS}
    nav2_core::nav2_core ${nav_msgs_TARGETS}
    nav2_costmap_2d::nav2_costmap_2d_core
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
  )
  target_include_directories(${global_planner_plugin_lib}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
      "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
  )
  pluginlib_export_plugin_description_file(nav2_core src/error_codes/planner_plugins.xml)

  install(TARGETS ${global_planner_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES src/error_codes/planner_plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )

  set(smoother_plugin_lib smoother)

  add_library(${smoother_plugin_lib} SHARED src/error_codes/smoother/smoother_error_plugin.cpp)
  target_link_libraries(${smoother_plugin_lib} PRIVATE
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    ${nav_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
  )
  target_include_directories(${smoother_plugin_lib}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
      "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
  )
  pluginlib_export_plugin_description_file(nav2_core src/error_codes/smoother_plugins.xml)

  install(TARGETS ${smoother_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES src/error_codes/smoother_plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )

  add_subdirectory(src/behavior_tree)
  add_subdirectory(src/planning)
  add_subdirectory(src/localization)
  add_subdirectory(src/system)
  add_subdirectory(src/system_failure)
  add_subdirectory(src/updown)
  add_subdirectory(src/waypoint_follower)
  add_subdirectory(src/route)
  add_subdirectory(src/gps_navigation)
  add_subdirectory(src/behaviors/wait)
  add_subdirectory(src/behaviors/spin)
  add_subdirectory(src/behaviors/backup)
  add_subdirectory(src/behaviors/drive_on_heading)
  add_subdirectory(src/behaviors/assisted_teleop)
  add_subdirectory(src/costmap_filters)
  add_subdirectory(src/error_codes)
  install(DIRECTORY maps models DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
