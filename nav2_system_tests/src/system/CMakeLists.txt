ament_add_test(test_bt_navigator
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=nav_to_pose_tester_node.py
    ASTAR=True
    CONTROLLER=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

ament_add_test(test_bt_navigator_with_wrong_init_pose
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_wrong_init_pose_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=nav_to_pose_tester_node.py
    ASTAR=True
    CONTROLLER=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

ament_add_test(test_bt_navigator_with_dijkstra
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=nav_to_pose_tester_node.py
    ASTAR=False
    CONTROLLER=dwb_core::DWBLocalPlanner
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

ament_add_test(test_bt_navigator_2
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=nav_to_pose_tester_node.py
    ASTAR=False
    CONTROLLER=dwb_core::DWBLocalPlanner
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

ament_add_test(test_dynamic_obstacle
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_with_obstacle_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    ADD_OBSTACLE=True
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=nav_to_pose_tester_node.py
    ASTAR=False
    CONTROLLER=dwb_core::DWBLocalPlanner
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

ament_add_test(test_nav_through_poses
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_with_obstacle_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    ADD_OBSTACLE=True
    BT_NAVIGATOR_XML=navigate_through_poses_w_replanning_and_recovery.xml
    TESTER=nav_through_poses_tester_node.py
    ASTAR=False
    CONTROLLER=dwb_core::DWBLocalPlanner
    PLANNER=nav2_navfn_planner::NavfnPlanner
)

# ament_add_test(test_multi_robot
#   GENERATE_RESULT_FOR_RETURN_CODE_ZERO
#   COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_multi_robot_launch.py"
#   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
#   TIMEOUT 180
#   ENV
#     TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
#     TEST_MAP=${PROJECT_SOURCE_DIR}/maps/map_circular.yaml
#     TEST_WORLD=${PROJECT_SOURCE_DIR}/worlds/world_only.model
#     TEST_URDF=${PROJECT_SOURCE_DIR}/urdf/turtlebot3_waffle.urdf
#     TEST_SDF=${PROJECT_SOURCE_DIR}/models/turtlebot3_waffle/model.sdf
#     BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
#     CONTROLLER=dwb_core::DWBLocalPlanner
#     PLANNER=nav2_navfn_planner::NavfnPlanner
#     TESTER=nav_to_pose_tester_node.py
# )
