add_library(map_loader SHARED
  map_loader.cpp
)

target_compile_definitions(map_loader PRIVATE "MAP_LOADER_BUILDING_DLL")

ament_target_dependencies(map_loader
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  SDL
  SDL_image
)

target_include_directories(map_loader PRIVATE
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
)

target_link_libraries(map_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
    ${library_name}
)

install(TARGETS
  map_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
