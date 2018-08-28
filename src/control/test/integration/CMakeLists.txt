ament_add_pytest_test(test_dwa_controller
  test_dwa_controller.py
  ENV TEST_LAUNCH_DIR=${TEST_LAUNCH_DIR}
  TIMEOUT 60)
