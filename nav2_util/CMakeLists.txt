cmake_minimum_required(VERSION 3.5)
project(nav2_util)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

nav2_package()

include_directories(include)

set(library_name ${PROJECT_NAME}_core)
add_library(${library_name} SHARED
  src/costmap.cpp
  src/node_utils.cpp
  src/lifecycle_service_client.cpp
  src/string_utils.cpp
  src/lifecycle_utils.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  nav2_msgs
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
)

add_library(map_lib SHARED
  src/map/map.c
  src/map/map_store.c
  src/map/map_range.c
  src/map/map_draw.c
  src/map/map_cspace.cpp
)

add_library(pf_lib SHARED
  src/pf/pf.c
  src/pf/pf_kdtree.c
  src/pf/pf_pdf.c
  src/pf/pf_vector.c
  src/pf/eig3.c
  src/pf/pf_draw.c
)

add_library(sensors_lib SHARED
  src/sensors/laser/laser.cpp
  src/sensors/laser/beam_model.cpp
  src/sensors/laser/likelihood_field_model.cpp
  src/sensors/laser/likelihood_field_model_prob.cpp
)
target_link_libraries(sensors_lib pf_lib)

add_library(motions_lib SHARED
  src/motion_model/omni_motion_model.cpp
  src/motion_model/differential_motion_model.cpp
)
target_link_libraries(motions_lib pf_lib)

add_library(map_loader SHARED
  src/map_loader/map_loader.cpp
)

ament_target_dependencies(map_loader
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  SDL
  SDL_image
)

target_link_libraries(map_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

add_executable(lifecycle_bringup
  src/lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})

install(TARGETS
  ${library_name}
  map_lib
  pf_lib
  sensors_lib
  motions_lib
  map_loader
  lifecycle_bringup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} pf_lib sensors_lib motions_lib map_lib map_loader)

ament_package()
