cmake_minimum_required(VERSION 3.5)

project(message_filters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} SHARED src/connection.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rcutils"
  "std_msgs"
  "builtin_interfaces"
  "sensor_msgs")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
DIRECTORY "include/"
DESTINATION include
)

find_package(python_cmake_module REQUIRED)
_ament_cmake_python_register_environment_hook()
install(
  DIRECTORY "src/${PROJECT_NAME}/"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
  if(TARGET ${PROJECT_NAME}-test_simple)
    target_link_libraries(${PROJECT_NAME}-test_simple ${PROJECT_NAME})
  endif()


  ament_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
    target_link_libraries(${PROJECT_NAME}-msg_cache_unittest ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
  if(TARGET ${PROJECT_NAME}-test_chain)
    target_link_libraries(${PROJECT_NAME}-test_chain ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-test_subscriber)
    target_link_libraries(${PROJECT_NAME}-test_subscriber ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
  if(TARGET ${PROJECT_NAME}-test_synchronizer)
    target_link_libraries(${PROJECT_NAME}-test_synchronizer ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
    target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_exact_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_fuzz test/test_fuzz.cpp SKIP_TEST)
  if(TARGET ${PROJECT_NAME}-test_fuzz)
    target_link_libraries(${PROJECT_NAME}-test_fuzz ${PROJECT_NAME})
  endif()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(msfpytests test
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 90
  )
endif()
ament_package()
