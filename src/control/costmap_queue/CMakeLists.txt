cmake_minimum_required(VERSION 3.5)
project(costmap_queue)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/costmap_queue.cpp
  src/limited_costmap_queue.cpp
)

set(dependencies
  costmap_2d
  rclcpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(mbq_test test/mbq_test.cpp)
  ament_target_dependencies(mbq_test ${dependencies})

  ament_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${PROJECT_NAME})
  ament_target_dependencies(utest ${dependencies})
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
