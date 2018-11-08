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

set(library_name nav2_dynamic_params)
set(executable_name ${library_name}_node)

add_library(${library_name}
  src/dynamic_params_validator.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(example_${executable_name}
  src/example_dynamic_params.cpp
)

ament_target_dependencies(example_${executable_name}
  ${dependencies}
)

target_link_libraries(example_${executable_name}
  nav2_dynamic_params
)

install(TARGETS ${library_name} example_${executable_name}
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
ament_export_libraries(${library_name})

ament_package()
