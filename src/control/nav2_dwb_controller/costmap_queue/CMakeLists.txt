cmake_minimum_required(VERSION 3.5)
project(costmap_queue)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall")

find_package(ament_cmake REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)

set(PROJECT_DEPENDENCIES
  costmap_2d
  rclcpp
  )

include_directories(include)

add_library(${PROJECT_NAME}
  src/costmap_queue.cpp
  src/limited_costmap_queue.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mbq_test test/mbq_test.cpp)
  ament_target_dependencies(mbq_test ${PROJECT_DEPENDENCIES})

  ament_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${PROJECT_NAME})
  ament_target_dependencies(utest ${PROJECT_DEPENDENCIES})
  #find_package(catkin REQUIRED COMPONENTS roslint)
  #roslint_cpp()
  #roslint_add_test()
endif (BUILD_TESTING)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
