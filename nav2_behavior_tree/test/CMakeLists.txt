ament_add_gtest(test_bt_conversions test_bt_conversions.cpp)
target_link_libraries(test_bt_conversions
  ${library_name}
  ${geometry_msgs_TARGETS}
)

function(plugin_add_test target filename plugin)
  ament_add_gtest(${target} ${filename})
  target_link_libraries(${target} ${geometry_msgs_TARGETS} nav2_util::nav2_util_core BT::behaviortree_cpp_v3 ${library_name} ${plugin})
  target_include_directories(${target}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/test>")
endfunction()

add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)
