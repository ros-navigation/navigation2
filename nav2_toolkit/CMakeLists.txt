cmake_minimum_required(VERSION 3.14)
project(nav2_pose_saver)

# C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include/nav2_pose_saver)




add_executable(pose_saver_node
  src/pose_saver_node.cpp
)
target_include_directories(pose_saver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(pose_saver_node yaml-cpp)


ament_target_dependencies(pose_saver_node
  rclcpp
  geometry_msgs
  std_srvs
  yaml-cpp
  ament_index_cpp
)

install(TARGETS
  pose_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
