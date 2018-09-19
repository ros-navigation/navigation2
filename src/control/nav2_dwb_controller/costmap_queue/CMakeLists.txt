cmake_minimum_required(VERSION 2.8.3)
project(costmap_queue)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS costmap_2d roscpp
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/costmap_queue.cpp
  src/limited_costmap_queue.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mbq_test test/mbq_test.cpp)
  target_link_libraries(mbq_test ${catkin_LIBRARIES})

  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${PROJECT_NAME} ${catkin_LIBRARIES})
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif (CATKIN_ENABLE_TESTING)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
