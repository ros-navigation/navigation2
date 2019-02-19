ament_add_test(test_simple_navigator
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 120
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    TEST_PARAMS=${PROJECT_SOURCE_DIR}/params/nav2_params.yaml
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    NAVIGATOR=simple
)

ament_add_test(test_bt_navigator
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 120
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
    TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/turtlebot3_ros2_demo.world
    TEST_PARAMS=${PROJECT_SOURCE_DIR}/params/nav2_params.yaml
    GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}/models
    NAVIGATOR=bt
    BT_NAVIGATOR_XML=simple_sequential.xml
)
