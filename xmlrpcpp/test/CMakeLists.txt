catkin_add_gtest(xmlrpcvalue_base64 xmlrpcvalue_base64.cpp)
if(TARGET xmlrpcvalue_base64)
  target_link_libraries(xmlrpcvalue_base64 xmlrpcpp)
endif()

catkin_add_gtest(test_base64 test_base64.cpp)
if(TARGET test_base64)
  target_link_libraries(test_base64 xmlrpcpp)
endif()

set_target_properties(test_base64 PROPERTIES COMPILE_FLAGS -std=c++11)
if(WIN32)
  # On Windows, gtest can be built as shared (dll) or static (lib),
  # to simplify the problem, here we require to match BUILD_SHARED_LIBS we used
  if(BUILD_SHARED_LIBS)
    set_target_properties(test_base64 PROPERTIES COMPILE_DEFINITIONS "GTEST_LINKED_AS_SHARED_LIBRARY=1")
  endif()
endif()

# Some of the tests that follow use boost threads.
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

add_library(test_fixtures test_fixtures.cpp)
target_link_libraries(test_fixtures xmlrpcpp ${Boost_LIBRARIES} ${GTEST_LIBRARIES})
set_target_properties(test_fixtures PROPERTIES EXCLUDE_FROM_ALL TRUE)

catkin_add_gtest(HelloTest HelloTest.cpp)
if(TARGET HelloTest)
  target_link_libraries(HelloTest xmlrpcpp ${Boost_LIBRARIES})
endif()

catkin_add_gtest(test_dispatch_live test_dispatch_live.cpp)
if(TARGET test_dispatch_live)
  target_link_libraries(test_dispatch_live xmlrpcpp test_fixtures ${Boost_LIBRARIES})
endif()

catkin_add_gtest(test_ulimit test_ulimit.cpp)
if(TARGET test_ulimit)
  target_link_libraries(test_ulimit xmlrpcpp test_fixtures ${Boost_LIBRARIES})
endif()

add_library(mock_socket mock_socket.cpp)
target_link_libraries(mock_socket ${GTEST_LIBRARIES})
set_target_properties(mock_socket PROPERTIES EXCLUDE_FROM_ALL TRUE)

catkin_add_gtest(test_client
  test_client.cpp
  ../src/XmlRpcClient.cpp
  ../src/XmlRpcValue.cpp
  ../src/XmlRpcUtil.cpp
  ../src/XmlRpcDispatch.cpp
  ../src/XmlRpcSource.cpp
  ../libb64/src/cdecode.c
  ../libb64/src/cencode.c
)
if(WIN32)
  # build everything from source, undefine ROS_BUILD_SHARED_LIBS to eliminate the dllimport/dllexport attributes
  set_target_properties(test_client PROPERTIES COMPILE_FLAGS -UROS_BUILD_SHARED_LIBS)
endif()

if(TARGET test_client)
  target_link_libraries(test_client mock_socket ${catkin_LIBRARIES})
  if(WIN32)
    target_link_libraries(test_client ws2_32)
  endif()
endif()

catkin_add_gtest(test_dispatch
  test_dispatch.cpp
  ../src/XmlRpcDispatch.cpp
  ../src/XmlRpcSource.cpp
  ../src/XmlRpcUtil.cpp
  ../libb64/src/cdecode.c
  ../libb64/src/cencode.c
)
if(WIN32)
  # build everything from source, undefine ROS_BUILD_SHARED_LIBS to eliminate the dllimport/dllexport attributes
  set_target_properties(test_dispatch PROPERTIES COMPILE_FLAGS -UROS_BUILD_SHARED_LIBS)
endif()

if(TARGET test_dispatch)
  target_link_libraries(test_dispatch mock_socket ${catkin_LIBRARIES})
  if(WIN32)
    target_link_libraries(test_dispatch ws2_32)
  endif()

  if(APPLE)
    set_target_properties(test_dispatch PROPERTIES
      LINK_FLAGS
      "-Wl,-alias,___wrap_poll,_poll"
    )
  elseif(UNIX)
    set_target_properties(test_dispatch PROPERTIES
      LINK_FLAGS
      "-Wl,--wrap=poll"
    )
  endif()
endif()

if(NOT WIN32)
  catkin_add_gtest(test_socket
    test_socket.cpp
    test_system_mocks.c
    ../src/XmlRpcSocket.cpp
    ../src/XmlRpcUtil.cpp
  )
  if(APPLE)
    set_target_properties(test_socket PROPERTIES
      LINK_FLAGS
      "-Wl,-alias,___wrap_accept,_accept -Wl,-alias,___wrap_bind,_bind -Wl,-alias,___wrap_close,_close -Wl,-alias,___wrap_connect,_connect -Wl,-alias,___wrap_getaddrinfo,_getaddrinfo -Wl,-alias,___wrap_getsockname,_getsockname -Wl,-alias,___wrap_listen,_listen -Wl,-alias,___wrap_read,_read -Wl,-alias,___wrap_setsockopt,_setsockopt -Wl,-alias,___wrap_select,_select -Wl,-alias,___wrap_select,_select$1050 -Wl,-alias,___wrap_socket,_socket -Wl,-alias,___wrap_write,_write -Wl,-alias,___wrap_fcntl,_fcntl -Wl,-alias,___wrap_freeaddrinfo,_freeaddrinfo"
    )
  elseif(UNIX)
    set_target_properties(test_socket PROPERTIES
      LINK_FLAGS
      "-Wl,--wrap=accept -Wl,--wrap=bind -Wl,--wrap=close -Wl,--wrap=connect -Wl,--wrap=getaddrinfo -Wl,--wrap=getsockname -Wl,--wrap=listen -Wl,--wrap=read -Wl,--wrap=setsockopt -Wl,--wrap=select -Wl,--wrap=socket -Wl,--wrap=write -Wl,--wrap=fcntl -Wl,--wrap=freeaddrinfo"
    )
  endif()
endif()

catkin_add_gtest(TestValues TestValues.cpp)
if(TARGET TestValues)
  target_link_libraries(TestValues xmlrpcpp)
endif()

catkin_add_gtest(test_util test_util.cpp)
if(TARGET test_util)
  target_link_libraries(test_util xmlrpcpp)
endif()

catkin_add_gtest(TestXml TestXml.cpp)
if(TARGET TestXml)
  target_link_libraries(TestXml xmlrpcpp)
endif()
