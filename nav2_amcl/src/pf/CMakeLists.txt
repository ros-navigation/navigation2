if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-folding-constant)
endif()

add_library(pf_lib SHARED
  pf.c
  pf_kdtree.c
  pf_pdf.c
  pf_vector.c
  eig3.c
  pf_draw.c
)

target_compile_definitions(pf_lib PRIVATE "NAV2_AMCL_PF_BUILDING_DLL")

install(TARGETS
  pf_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
