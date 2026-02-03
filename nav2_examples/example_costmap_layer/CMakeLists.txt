cmake_minimum_required(VERSION 3.8)
project(example_costmap_layer)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME}_plugin)

set(dependencies
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
)

add_library(${library_name} SHARED
  src/gradient_layer.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name} PUBLIC ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d example_costmap_layer_plugin.xml)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES example_costmap_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  nav2_costmap_2d
  nav2_util
  rclcpp
)
ament_export_targets(${library_name})
ament_package()
