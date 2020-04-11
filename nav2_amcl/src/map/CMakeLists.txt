add_library(map_lib SHARED
  map.c
  map_store.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

target_compile_definitions(map_lib PRIVATE "NAV2_AMCL_MAP_BUILDING_DLL")

install(TARGETS
  map_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
