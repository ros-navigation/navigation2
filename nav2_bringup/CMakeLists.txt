cmake_minimum_required(VERSION 3.20)
project(nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

nav2_package()

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR launch)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY graphs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
