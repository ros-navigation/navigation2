cmake_minimum_required(VERSION 3.5)

project(nav2_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(nav2_common)

ament_package(
  CONFIG_EXTRAS "nav2_common-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
