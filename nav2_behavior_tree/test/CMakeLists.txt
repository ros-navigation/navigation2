ament_add_gtest(test_bt_conversions test_bt_conversions.cpp)
ament_target_dependencies(test_bt_conversions ${dependencies})

ament_add_gtest(test_footprint_utils test_footprint_utils.cpp)
ament_target_dependencies(test_footprint_utils ${dependencies})

ament_add_gtest(test_classified_path_markers test_classified_path_markers.cpp)
ament_target_dependencies(test_classified_path_markers ${dependencies})

add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)
