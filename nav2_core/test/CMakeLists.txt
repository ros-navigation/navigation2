# NavigatorMuxer test
nav2_add_gtest(test_navigator_muxer test_navigator_muxer.cpp)
target_link_libraries(test_navigator_muxer
  nav2_core
)