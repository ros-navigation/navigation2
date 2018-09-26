cmake_minimum_required(VERSION 2.8.3)
project(dwb_local_planner)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwb_local_planner debug_dwb_local_planner
    CATKIN_DEPENDS
        costmap_2d dwb_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav_core2
        nav_msgs pluginlib roscpp sensor_msgs tf visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dwb_local_planner src/dwb_local_planner.cpp
                              src/backwards_compatibility.cpp
                              src/publisher.cpp
                              src/illegal_trajectory_tracker.cpp
)
target_link_libraries(dwb_local_planner ${catkin_LIBRARIES})
add_dependencies(dwb_local_planner ${catkin_EXPORTED_TARGETS})

add_library(debug_dwb_local_planner src/debug_dwb_local_planner.cpp)
target_link_libraries(debug_dwb_local_planner ${catkin_LIBRARIES} dwb_local_planner)
add_dependencies(debug_dwb_local_planner ${catkin_EXPORTED_TARGETS})

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node ${catkin_LIBRARIES} debug_dwb_local_planner)
add_dependencies(planner_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS planner_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS dwb_local_planner debug_dwb_local_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)
