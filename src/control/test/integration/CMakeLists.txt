ament_add_pytest_test(test_dwa_controller
  test_dwa_controller.py
  ENV CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  TIMEOUT 60)
