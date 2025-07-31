cmake_minimum_required(VERSION 3.5)

project(nav2_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)

ament_python_install_package(nav2_common)

ament_package(
  CONFIG_EXTRAS "nav2_common-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_rewritten_yaml test/test_rewritten_yaml.py
  )
  ament_add_pytest_test(test_launch_config_as_bool test/test_launch_config_as_bool.py
  )
endif()
