cmake_minimum_required(VERSION 3.5)
project(nav2_constrained_smoother)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_CXX_STANDARD 17)

if(${CERES_VERSION} VERSION_LESS_EQUAL 2.0.0)
  add_definitions(-DUSE_OLD_CERES_API)
endif()

nav2_package()

set(library_name nav2_constrained_smoother)

include_directories(
  include
  ${CERES_INCLUDES}
)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_costmap_2d
  nav2_util
  nav2_core
  pluginlib
)

add_library(${library_name} SHARED src/constrained_smoother.cpp)
target_link_libraries(${library_name} ${CERES_LIBRARIES})
# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(${library_name} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(
  TARGETS
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_constrained_smoother.xml)

ament_package()
