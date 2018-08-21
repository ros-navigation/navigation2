cmake_minimum_required(VERSION 3.5)
project(nav2_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include
  )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "../libs/")

link_directories(${CMAKE_BINARY_DIR}/../libs)

add_library(nav2_robot STATIC
  src/RosRobot.cpp
  )
