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

add_library(nav2_constrained_smoother SHARED src/constrained_smoother.cpp)
add_library(nav2_constrained_smoother::nav2_constrained_smoother ALIAS nav2_constrained_smoother)
target_link_libraries(nav2_constrained_smoother PUBLIC Ceres::ceres)
# This is a temporary hack:
# 1) Eigen 3.4.0 has compiler warnings in gcc12 -Wmaybe-unitialized
# 2) Ceres doesn't set EIGEN as a system dependency properly
# 3) Nav2 treats unitialialized variables as errors
# It can be removed when a new Eigen+Ceres versions get into ubuntu mainline
# https://discourse.cmake.org/t/declare-imported-targets-as-system-to-suppress-warnings/555
get_target_property(ceres_include_dirs Ceres::ceres INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(Ceres::ceres SYSTEM INTERFACE ${ceres_include_dirs})

# prevent pluginlib from using boost
target_compile_definitions(nav2_constrained_smoother PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(nav2_constrained_smoother PUBLIC ${dependencies})
target_include_directories(nav2_constrained_smoother
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

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
  nav2_constrained_smoother
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

ament_export_libraries(nav2_constrained_smoother)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_constrained_smoother.xml)

ament_package()
