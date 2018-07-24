cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include
  )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "../libs/")

add_library(robot STATIC
  src/RosRobot.cpp
  )
