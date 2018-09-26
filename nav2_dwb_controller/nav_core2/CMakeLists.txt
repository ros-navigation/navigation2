cmake_minimum_required(VERSION 2.8.3)
project(nav_core2)

find_package(catkin REQUIRED
    COMPONENTS costmap_2d nav_2d_msgs tf
)

catkin_package(
    CATKIN_DEPENDS costmap_2d nav_2d_msgs tf
    INCLUDE_DIRS include
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
