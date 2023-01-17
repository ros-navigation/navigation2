ament_add_gtest(test_bt_conversions test_bt_conversions.cpp)
ament_target_dependencies(test_bt_conversions ${dependencies})

include_directories(.)

add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)

install(DIRECTORY utils
        DESTINATION include/)

ament_export_include_directories(utils)
