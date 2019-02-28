
#include_directories( ${PROJECT_SOURCE_DIR}/src )
#include_directories( ${PROJECT_SOURCE_DIR}/test )

ament_add_gtest(test_motion_primitives test_motion_primitives.cpp)

ament_target_dependencies( test_motion_primitives
  ${dependencies}
)