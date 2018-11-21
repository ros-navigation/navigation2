cmake_minimum_required(VERSION 3.5)
project(nav2_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(costmap_lib SHARED
  src/costmap.cpp
)

ament_target_dependencies(costmap_lib
  rclcpp
  nav2_msgs
  tf2
  nav_msgs
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

add_library(motions_lib SHARED
  src/motion_model/omni_motion_model.cpp
  src/motion_model/differential_motion_model.cpp
)

add_library(map_loader SHARED
  src/map_loader/map_loader.cpp
)

ament_target_dependencies(map_loader
  geometry_msgs
  tf2
  nav_msgs
  SDL
  SDL_image
)

target_link_libraries(map_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

install(TARGETS
  costmap_lib
  map_lib
  pf_lib
  sensors_lib
  motions_lib
  map_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

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
ament_export_libraries(costmap_lib pf_lib sensors_lib motions_lib map_lib map_loader)

ament_package()
