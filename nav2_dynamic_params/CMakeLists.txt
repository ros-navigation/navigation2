cmake_minimum_required(VERSION 3.5)
project(nav2_dynamic_params)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
)

add_library(nav2_dynamic_params
  src/dynamic_params_validator.cpp
)

ament_target_dependencies(nav2_dynamic_params
  ${dependencies}
)

add_executable(example_nav2_dynamic_params
  src/example_dynamic_params.cpp
)

ament_target_dependencies(example_nav2_dynamic_params
  ${dependencies}
)

target_link_libraries(example_nav2_dynamic_params
  nav2_dynamic_params
)

install(TARGETS nav2_dynamic_params example_nav2_dynamic_params
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
  # TODO(bpwilcox): Port/Add unit and component tests
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_dynamic_params)

ament_package()
