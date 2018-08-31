set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)
ADD_DEFINITIONS( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

add_subdirectory(unit)
add_subdirectory(integration)
