set(PRAY_SOURCES
  debug.hpp
  ext.cpp
  ext.hpp
  image.hpp
  image.cpp
  main.cpp
  math.hpp
  scene.cpp
  scene.hpp
  simd.hpp
  cpu_tracer.hpp
  cpu_tracer.impl.hpp
  )

if (WITH_CUDA)
  cuda_compile(cuda_files
    cuda_renderer.hpp
    cuda_renderer.cu)
  list(APPEND PRAY_SOURCES ${cuda_files})
endif()

add_executable(pray ${PRAY_SOURCES})


set_target_properties(pray PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	target_link_libraries(pray "-lc++experimental")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_link_libraries(pray "-lstdc++fs")
endif()
if (WITH_OMP)
  target_compile_options(pray PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(pray ${OpenMP_CXX_FLAGS})
elseif(NOT MSVC)
  target_compile_options(pray PRIVATE -Wno-error=unknown-pragmas)
endif()
