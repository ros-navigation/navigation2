cmake_minimum_required(VERSION 3.5)
project(env_model)

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
find_package(env_model_msgs)

include_directories(
	include
	${ament_INCLUDE_DIRS}
)

add_library(env_mods
	src/occupancy_grid.cpp
	#src/grid_map.cpp
	src/env_model_loader.cpp
	src/utilities/cacheddistmap.cpp
)
ament_target_dependencies(env_mods rclcpp nav_msgs env_model_msgs)
target_link_libraries(env_mods
	${ament_LIBRARIES}
)


add_executable(env_model src/env_model_ros.cpp)
ament_target_dependencies(env_model rclcpp nav_msgs env_model_msgs)
target_link_libraries(env_model
	env_mods
	${ament_LIBRARIES}
)

add_executable(env_model_client src/env_model_client.cpp)
ament_target_dependencies(env_model_client rclcpp env_model_msgs)
target_link_libraries(env_model_client
	env_mods
	${ament_LIBRARIES}
)

install(TARGETS env_mods env_model env_model_client DESTINATION lib/${PROJECT_NAME})


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

ament_package()

