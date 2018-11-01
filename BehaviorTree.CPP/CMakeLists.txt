cmake_minimum_required(VERSION 3.5)
project(behavior_tree_core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -Werror=return-type")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(GTest)

include_directories(${GTEST_INCLUDE_DIRS})
if(NOT GTEST_FOUND)
  message(WARNING " GTest not found!")
endif(NOT GTEST_FOUND)

set(BT_Source
  src/action_node.cpp
  src/basic_types.cpp
  src/decorator_node.cpp
  src/condition_node.cpp
  src/control_node.cpp
  src/exceptions.cpp
  src/leaf_node.cpp
  src/tick_engine.cpp
  src/tree_node.cpp
  src/bt_factory.cpp
  src/behavior_tree.cpp
  src/xml_parsing.cpp
  src/shared_library.cpp
  src/shared_library_UNIX.cpp
  src/decorators/inverter_node.cpp
  src/decorators/repeat_node.cpp
  src/decorators/retry_node.cpp
  src/decorators/timeout_node.cpp
  src/controls/parallel_node.cpp
  src/controls/sequence_node.cpp
  src/controls/sequence_star_node.cpp
  src/controls/fallback_node.cpp
  src/controls/fallback_star_node.cpp
  src/loggers/bt_cout_logger.cpp
  src/loggers/bt_file_logger.cpp
  src/loggers/bt_minitrace_logger.cpp
  3rdparty/tinyXML2/tinyxml2.cpp
  3rdparty/minitrace/minitrace.cpp
)
  
#set_source_files_properties(tinyxml2.cpp PROPERTIES COMPILE_FLAGS -fvisiblity=hidden)

include_directories(include 3rdparty/)

set(BEHAVIOR_TREE_LIBRARIES behavior_tree_core)

find_package(ZMQ)

if(ZMQ_FOUND)
  message(STATUS "ZeroMQ found.")
  add_definitions( -DZMQ_FOUND )
  set(BT_Source ${BT_Source} src/loggers/bt_zmq_publisher.cpp )
  set(BEHAVIOR_TREE_LIBRARIES ${BEHAVIOR_TREE_LIBRARIES} zmq)
else()
  message(WARNING "ZeroMQ NOT found. Skipping the build of [PublisherZMQ] and [bt_recorder].")
endif()

######################################################
# COMPILING LIBRARY
######################################################
add_library(behavior_tree_core SHARED ${BT_Source} )
target_compile_options(behavior_tree_core PRIVATE "-fPIC")
target_link_libraries(behavior_tree_core dl)

######################################################
# TESTS
######################################################
set(BT_Tests
  gtest/src/action_test_node.cpp
  gtest/src/condition_test_node.cpp
  gtest/gtest_tree.cpp
  gtest/gtest_sequence.cpp
  gtest/gtest_parallel.cpp
  gtest/gtest_fallback.cpp
  gtest/gtest_factory.cpp
  3rdparty/tinyXML2/tinyxml2.cpp
  3rdparty/minitrace/minitrace.cpp
)

include_directories(gtest/include)

ament_add_gtest(behavior_tree_core_test ${BT_Tests} )

target_link_libraries(behavior_tree_core_test
	${BEHAVIOR_TREE_LIBRARIES} crossdoor_nodes
	${catkin_LIBRARIES}
)

######################################################
# COMPILING EXAMPLES and TOOLS
######################################################
add_executable(bt_log_cat         tools/bt_log_cat.cpp )
target_link_libraries(bt_log_cat  ${BEHAVIOR_TREE_LIBRARIES} )

if(ZMQ_FOUND)
  add_executable(bt_recorder         tools/bt_recorder.cpp )
  target_link_libraries(bt_recorder  ${BEHAVIOR_TREE_LIBRARIES} )
endif()

add_executable(bt_plugin_manifest         tools/bt_plugin_manifest.cpp )
target_link_libraries(bt_plugin_manifest  ${BEHAVIOR_TREE_LIBRARIES} )

add_subdirectory(sample_nodes)
add_subdirectory(examples)

ament_export_include_directories(include)

install(TARGETS behavior_tree_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY 3rdparty/non_std/
  DESTINATION include/non_std
)

install(DIRECTORY 3rdparty/tinyXML2/
  DESTINATION include/tinyXML2
)

ament_export_libraries(${BEHAVIOR_TREE_LIBRARIES})
ament_package()
