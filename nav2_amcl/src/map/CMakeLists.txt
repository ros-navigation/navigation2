add_library(map_lib SHARED
  map.c
  map_store.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

install(TARGETS
  map_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
