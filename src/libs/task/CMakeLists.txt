cmake_minimum_required(VERSION 3.5)
project(task)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include
  /usr/local/include/yasmine
  /usr/local/include/yasmine/include
  /usr/local/include/yasmine/include_impl
  /usr/local/include/yasmine/essentials/include
  )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "../libs/")

add_library(task STATIC
  src/TaskExecutionStateMachine.cpp
  )
