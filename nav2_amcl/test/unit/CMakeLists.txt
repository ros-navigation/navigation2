include_directories( ${PROJECT_SOURCE_DIR}/src )

ament_add_gtest(test_localization_amcl test_localization_amcl.cpp)
ament_target_dependencies( test_localization_amcl
  ${dependencies}
)

