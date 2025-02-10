find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
  )
endforeach()
