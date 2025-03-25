cmake_minimum_required(VERSION 3.5)
project(costmap_queue)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

nav2_package()

add_library(${PROJECT_NAME} SHARED
  src/costmap_queue.cpp
  src/limited_costmap_queue.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}
  nav2_costmap_2d
)
# target_link_libraries(${PROJECT_NAME} PUBLIC
#   nav2_costmap_2d::nav2_costmap_2d_core
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  find_package(rclcpp REQUIRED)

  ament_find_gtest()

  ament_add_gtest(mbq_test test/mbq_test.cpp)
  target_link_libraries(mbq_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(utest test/utest.cpp)
  ament_target_dependencies(utest
    nav2_costmap_2d
    rclcpp
  )
  target_link_libraries(utest
    ${PROJECT_NAME}
    # nav2_costmap_2d::nav2_costmap_2d_core
    # rclcpp::rclcpp
  )
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_costmap_2d)
ament_export_targets(${PROJECT_NAME})

ament_package()
