add_library(motions_lib SHARED
  omni_motion_model.cpp
  differential_motion_model.cpp
)
target_link_libraries(motions_lib pf_lib)
