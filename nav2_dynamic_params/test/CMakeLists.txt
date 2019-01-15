add_executable(test_dynamic_params_helper test_dynamic_params_helper.cpp)

ament_target_dependencies(test_dynamic_params_helper ${dependencies})

target_link_libraries(test_dynamic_params_helper
  nav2_dynamic_params
)

install(TARGETS test_dynamic_params_helper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_add_gtest(test_dynamic_params_validator test_dynamic_params_validator.cpp)

ament_target_dependencies(test_dynamic_params_validator ${dependencies})

target_link_libraries(test_dynamic_params_validator
  nav2_dynamic_params
)

ament_add_gtest_executable(test_dynamic_params_client_exec test_dynamic_params_client.cpp)

ament_target_dependencies(test_dynamic_params_client_exec ${dependencies})

target_link_libraries(test_dynamic_params_client_exec
  nav2_dynamic_params
)

ament_add_test(test_dynamic_params_client
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/dynamic_params_test.launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_dynamic_params_client_exec>
)