cmake_minimum_required(VERSION 3.5)
project(map_server)

set(YAML_CPP_ROS2_LIBRARIES $ENV{HOME}/ros2_ws/ros2_ws/install/rviz_yaml_cpp_vendor/opt/rviz_yaml_cpp_vendor/lib/libyaml-cpp.so)
set(YAML_CPP_ROS2_INCLUDE_DIRS $ENV{HOME}/ros2_ws/ros2_ws/install/rviz_yaml_cpp_vendor/opt/rviz_yaml_cpp_vendor/include)

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
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(example_interfaces REQUIRED)

if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

include_directories(
	include
	${BULLET_INCLUDE_DIRS}
	${ament_INCLUDE_DIRS}
	${SDL_INCLUDE_DIR}
	${SDL_IMAGE_INCLUDE_DIRS}
	${YAML_CPP_ROS2_INCLUDE_DIRS}
	
)


add_library(map_reps
	src/map_reps/occupancy_grid.cpp
	src/map_reps/map_loader.cpp
	#src/map_reps/gridmap.cpp

)
ament_target_dependencies(map_reps rclcpp nav_msgs)
target_link_libraries(map_reps
	${BULLET_LIBRARIES}
	${ament_LIBRARIES}
	${SDL_LIBRARY}
	${SDL_IMAGE_LIBRARIES}
	${YAML_CPP_ROS2_LIBRARIES}
)


add_executable(map_server src/map_server.cpp)
ament_target_dependencies(map_server rclcpp nav_msgs)
target_link_libraries(map_server
	map_reps
	${ament_LIBRARIES}
)



install(TARGETS map_reps map_server DESTINATION lib/${PROJECT_NAME})


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)



ament_package()


