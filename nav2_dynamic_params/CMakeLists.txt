cmake_minimum_required(VERSION 3.5)
project(nav2_dynamic_params)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
)

add_library(nav2_dynamic_params SHARED
  src/dynamic_params_validator.cpp
)

ament_target_dependencies(nav2_dynamic_params
  ${dependencies}
)

add_executable(example_nav2_dynamic_params_validator
  src/example_dynamic_params_validator.cpp
)

ament_target_dependencies(example_nav2_dynamic_params_validator
  ${dependencies}
)

target_link_libraries(example_nav2_dynamic_params_validator
  nav2_dynamic_params
)

add_executable(example_nav2_dynamic_params_client
  src/example_dynamic_params_client.cpp
)

ament_target_dependencies(example_nav2_dynamic_params_client
  ${dependencies}
)

target_link_libraries(example_nav2_dynamic_params_client
  nav2_dynamic_params
)

install(TARGETS nav2_dynamic_params example_nav2_dynamic_params_validator example_nav2_dynamic_params_client
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

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_dynamic_params)

ament_package()
