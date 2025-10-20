ament_add_gtest(test_amcl_convert_map test_amcl_convert_map.cpp)
ament_target_dependencies(test_amcl_convert_map
    ${dependencies}
)
target_link_libraries(test_amcl_convert_map
  ${library_name}
)
