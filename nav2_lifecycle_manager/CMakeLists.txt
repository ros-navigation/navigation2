cmake_minimum_required(VERSION 3.5)
project(nav2_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  nav2_util::nav2_util_core
  bondcpp::bondcpp
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  diagnostic_updater::diagnostic_updater
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  rclcpp_components::component
)

add_executable(lifecycle_manager
  src/main.cpp
)
target_include_directories(lifecycle_manager
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(lifecycle_manager PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

rclcpp_components_register_nodes(${library_name} "nav2_lifecycle_manager::LifecycleManager")

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(nav2_util bondcpp rclcpp std_srvs diagnostic_updater)
ament_export_targets(${library_name})

ament_package()
