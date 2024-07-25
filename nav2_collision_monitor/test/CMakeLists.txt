# Kinematics test
ament_add_gtest(kinematics_test kinematics_test.cpp)
ament_target_dependencies(kinematics_test
  ${dependencies}
)
target_link_libraries(kinematics_test
  ${monitor_library_name}
)

# Data sources test
ament_add_gtest(sources_test sources_test.cpp)
ament_target_dependencies(sources_test
  ${dependencies}
)
target_link_libraries(sources_test
  ${monitor_library_name}
)

# Polygon shapes test
ament_add_gtest(polygons_test polygons_test.cpp)
ament_target_dependencies(polygons_test
  ${dependencies}
)
target_link_libraries(polygons_test
  ${monitor_library_name}
)

# Velocity Polygon test
ament_add_gtest(velocity_polygons_test velocity_polygons_test.cpp)
ament_target_dependencies(velocity_polygons_test
  ${dependencies}
)
target_link_libraries(velocity_polygons_test
  ${monitor_library_name}
)

# Collision Monitor node test
ament_add_gtest(collision_monitor_node_test collision_monitor_node_test.cpp)
ament_target_dependencies(collision_monitor_node_test
  ${dependencies}
)
target_link_libraries(collision_monitor_node_test
  ${monitor_library_name}
)
# Collision Detector node test
ament_add_gtest(collision_detector_node_test collision_detector_node_test.cpp)
ament_target_dependencies(collision_detector_node_test
  ${dependencies}
)
target_link_libraries(collision_detector_node_test
  ${detector_library_name}
)