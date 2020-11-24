set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

add_subdirectory(unit)
add_subdirectory(component)
add_subdirectory(map_saver_cli)
