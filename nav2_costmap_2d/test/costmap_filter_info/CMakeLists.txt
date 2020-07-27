set(costmap_filter_info_pub_executable costmap_filter_info_pub)

add_executable(${costmap_filter_info_pub_executable}
  costmap_filter_info_pub.cpp)

set(costmap_filter_info_pub_deps
  rclcpp
  nav_msgs
  nav2_msgs)

ament_target_dependencies(${costmap_filter_info_pub_executable}
  ${costmap_filter_info_pub_deps})
