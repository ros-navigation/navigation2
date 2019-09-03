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

install(TARGETS
  pf_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
