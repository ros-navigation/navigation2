add_executable(dummy_planner_node
  src/dummy_planner/main.cpp
  src/dummy_planner/dummy_planner.cpp
)
target_link_libraries(dummy_planner_node PRIVATE
  nav2_behavior_tree::nav2_behavior_tree
  rclcpp::rclcpp
)
target_include_directories(dummy_planner_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
