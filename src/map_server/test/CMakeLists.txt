set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)
add_subdirectory(unit)
add_subdirectory(integration)

# Install launch files.
install(DIRECTORY
  test_launch_files
  DESTINATION share/${PROJECT_NAME}/
)