ament_add_test(test_route
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_route_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
    TESTER=tester_node.py
    ASTAR=True
    PATH_HANDLER=nav2_controller::SimplePathHandler
    CONTROLLER=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    PLANNER=nav2_navfn_planner::NavfnPlanner
)
