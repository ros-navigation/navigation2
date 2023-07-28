ament_add_gtest(test_bt_utils test_bt_utils.cpp)
ament_target_dependencies(test_bt_utils ${dependencies})

include_directories(.)

add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)
