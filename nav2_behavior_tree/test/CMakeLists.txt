ament_add_gtest(test_bt_utils test_bt_utils.cpp)
ament_target_dependencies(test_bt_utils ${dependencies})

ament_add_gtest(test_json_utils test_json_utils.cpp)
target_link_libraries(test_json_utils
  ${library_name}
  ${geometry_msgs_TARGETS})

include_directories(.)



add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)
