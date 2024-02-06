find_package(nav2_controller REQUIRED)

# Tests for Graceful Motion Controller
ament_add_gtest(test_graceful_motion_controller
  test_graceful_motion_controller.cpp
)
ament_target_dependencies(test_graceful_motion_controller
  ${dependencies}
  nav2_controller
)
target_link_libraries(test_graceful_motion_controller
  ${library_name}
)

# Egopolar test
ament_add_gtest(test_egopolar
  test_egopolar.cpp
)
ament_target_dependencies(test_egopolar
  geometry_msgs tf2_geometry_msgs angles
)
