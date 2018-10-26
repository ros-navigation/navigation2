cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    voxel_grid
  CATKIN_DEPENDS
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(voxel_grid src/voxel_grid.cpp)
add_dependencies(voxel_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_grid ${catkin_LIBRARIES})

install(TARGETS voxel_grid
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(voxel_grid_tests test/voxel_grid_tests.cpp)
  target_link_libraries(voxel_grid_tests
    voxel_grid
    ${catkin_LIBRARIES}
  )
endif()
