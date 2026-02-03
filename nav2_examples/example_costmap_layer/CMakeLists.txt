cmake_minimum_required(VERSION 3.8)
project(example_costmap_layer)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

nav2_package()

# Create shared library
set(library_name ${PROJECT_NAME}_plugin)
add_library(${library_name} SHARED
  src/gradient_layer.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
)

# Install library
install(TARGETS ${library_name}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install plugin descriptor XML
install(FILES example_costmap_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d example_costmap_layer_plugin.xml)

# Export package information
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
