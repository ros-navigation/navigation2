# Bresenham2D corner cases test
ament_add_gtest(costmap_bresenham_2d costmap_bresenham_2d.cpp)
target_link_libraries(costmap_bresenham_2d
  nav2_costmap_2d_core
)

# OrderLayer for checking Costmap2D plugins API calling order
add_library(order_layer SHARED
            order_layer.cpp)
ament_target_dependencies(order_layer
  ${dependencies}
)
install(TARGETS
  order_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Costmap2D plugins API calling order test
ament_add_gtest(plugin_api_order plugin_api_order.cpp)
target_link_libraries(plugin_api_order
  nav2_costmap_2d_core
)
