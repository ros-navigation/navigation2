cmake_minimum_required(VERSION 3.5)
project(nav2_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(BFL REQUIRED orocos-bfl)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  pluginlib
  nav2_util
  nav_2d_utils
  message_filters
  sensor_msgs
  tf2_ros
  tf2
  nav2_msgs
)

include_directories(
  include
  ${BFL_INCLUDE_DIRS}/bfl
)

add_library(sample_motion_model SHARED src/plugins/sample_motion_model.cpp)
ament_target_dependencies(sample_motion_model ${dependencies})

add_library(matcher2d SHARED src/plugins/matcher2d.cpp)
ament_target_dependencies(matcher2d ${dependencies})

add_library(solver SHARED src/plugins/solver.cpp src/custom_particle_filter.cpp)
ament_target_dependencies(solver ${dependencies})
target_link_libraries(solver orocos-bfl)

set(executable_name nav2_localization)
add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/nav2_localization.cpp
)

target_include_directories(${library_name} PRIVATE src/include)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_directories(solver PUBLIC ${BFL_LIBRARY_DIRS})

install(TARGETS sample_motion_model matcher2d solver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_localization plugins.xml)

ament_package()
