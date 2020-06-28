
# ament_add_gtest_executable(some_integration_test_exec
#   some_integration_test.cpp
# )
# ament_target_dependencies(some_integration_test_exec
#   ${dependencies}
# )
# target_link_libraries(some_integration_test_exec
#   bt_waypoint_follower_core
# )

# ament_add_test(some_integration_test
#   GENERATE_RESULT_FOR_RETURN_CODE_ZERO
#   COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/bt_waypoint_follower_tests_launch.py"
#   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
#   ENV
#     TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
#     TEST_EXECUTABLE=$<TARGET_FILE:some_integration_test_exec>
# )
