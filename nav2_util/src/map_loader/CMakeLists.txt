add_library(map_loader SHARED
  map_loader.cpp
)

ament_target_dependencies(map_loader
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  SDL
  SDL_image
)

target_link_libraries(map_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

install(TARGETS
  map_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
