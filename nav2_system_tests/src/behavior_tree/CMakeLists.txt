set(test_behavior_tree_exec test_behavior_tree_node)

ament_add_gtest_executable(${test_behavior_tree_exec}
  test_behavior_tree_node.cpp
  behavior_tree_tester.cpp
)

ament_target_dependencies(${test_behavior_tree_exec}
  ${dependencies}
)

ament_add_test(test_behavior_tree
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_behavior_tree_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_EXECUTABLE=$<TARGET_FILE:${test_behavior_tree_exec}>
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    BT_NAVIGATOR_XML=navigate_w_replanning_and_recovery.xml
)