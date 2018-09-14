cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(rviz_yaml_cpp_vendor REQUIRED)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
)

set(executable_name map_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/map_reps/occ_grid_server.cpp
  src/map_reps/map_factory.cpp
  src/map_server_ros.cpp
)

set(dependencies
  rclcpp
  nav_msgs
  rviz_yaml_cpp_vendor
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

target_link_libraries(${library_name}
  ${BULLET_LIBRARIES}
  ${SDL_LIBRARY}
	${SDL_IMAGE_LIBRARIES}
)
install(TARGETS ${executable_name} ${library_name}
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
endif()

ament_export_include_directories(include)

ament_package()
