add_library(motions_lib SHARED
  omni_motion_model.cpp
  differential_motion_model.cpp
  motion_model.cpp
)
target_link_libraries(motions_lib pf_lib)

install(TARGETS
  motions_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
