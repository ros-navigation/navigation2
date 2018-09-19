cmake_minimum_required(VERSION 2.8.3)
project(dwb_plugins)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS angles dwb_local_planner dynamic_reconfigure nav_2d_msgs nav_2d_utils nav_core2 pluginlib roscpp)

generate_dynamic_reconfigure_options(cfg/KinematicParams.cfg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS angles dwb_local_planner dynamic_reconfigure nav_2d_msgs nav_2d_utils nav_core2 pluginlib roscpp
    LIBRARIES simple_goal_checker stopped_goal_checker standard_traj_generator
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(simple_goal_checker src/simple_goal_checker.cpp)
target_link_libraries(simple_goal_checker ${catkin_LIBRARIES})
add_dependencies(simple_goal_checker ${catkin_EXPORTED_TARGETS} ${dwb_plugins_EXPORTED_TARGETS})

add_library(stopped_goal_checker src/stopped_goal_checker.cpp)
target_link_libraries(stopped_goal_checker simple_goal_checker ${catkin_LIBRARIES})
add_dependencies(stopped_goal_checker ${catkin_EXPORTED_TARGETS} ${dwb_plugins_EXPORTED_TARGETS})

add_library(standard_traj_generator
            src/standard_traj_generator.cpp
            src/limited_accel_generator.cpp
            src/kinematic_parameters.cpp
            src/xy_theta_iterator.cpp)
target_link_libraries(standard_traj_generator ${catkin_LIBRARIES})
add_dependencies(standard_traj_generator ${catkin_EXPORTED_TARGETS} ${dwb_plugins_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(vtest test/velocity_iterator_test.cpp)

  find_package(rostest REQUIRED)
  add_executable(goal_checker test/goal_checker.cpp)
  target_link_libraries(goal_checker simple_goal_checker stopped_goal_checker ${GTEST_LIBRARIES})
  add_rostest(test/goal_checker.launch DEPENDENCIES goal_checker)

  add_executable(twist_gen_test test/twist_gen.cpp)
  target_link_libraries(twist_gen_test standard_traj_generator ${GTEST_LIBRARIES})
  add_rostest(test/twist_gen.launch DEPENDENCIES twist_gen_test)

  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif (CATKIN_ENABLE_TESTING)

install(TARGETS simple_goal_checker stopped_goal_checker standard_traj_generator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
