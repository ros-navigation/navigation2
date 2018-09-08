cmake_minimum_required(VERSION 3.5)
project(MissionPlanner_BT)
aux_source_directory(./src/ SRC_LIST)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(BTpp REQUIRED)

include_directories(include ${BTpp_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SRC_LIST})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${BTpp_LIBRARIES})

