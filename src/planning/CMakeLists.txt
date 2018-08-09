cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(task REQUIRED)

include_directories(
  include
  ../task/include
  ../world_model/include
  ../costmap/include
  )

link_directories(${CMAKE_BINARY_DIR}/../libs)

# add_executable(astar_planner
#   src/AStarPlanner.cpp
#   src/main.cpp
#   )

add_executable(dijkstra_planner
  src/DijkstraPlanner.cpp
  src/Navfn.cpp
  src/main.cpp
  )

target_link_libraries(
  dijkstra_planner
  # astar_planner
)

# ament_target_dependencies(astar_planner
#   rclcpp
#   std_msgs
#   nav2_msgs
#   )

ament_target_dependencies(dijkstra_planner
  rclcpp
  std_msgs
  nav2_msgs
  task
)

install(TARGETS
  dijkstra_planner
  # astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_include_directories(include)

ament_package()
