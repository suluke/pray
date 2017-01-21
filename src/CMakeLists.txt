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
  ray.hpp
  sse_ray.hpp
  cpu_tracer.hpp
  cpu_tracer.impl.hpp
  cpu_pathtracer.hpp
  cpu_pathtracer.impl.hpp
  sampler.hpp
  dummy_acceleration.hpp
  bih.hpp
  bih.impl.hpp
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
  find_library(LIBCXX_EXPERIMENTAL NAMES "c++experimental" "libc++experimental" HAS_LIBCXX11)
  if(${LIBCXX_EXPERIMENTAL} STREQUAL "LIBCXX_EXPERIMENTAL-NOTFOUND")
    target_link_libraries(pray "-lstdc++fs")
  else()
    target_link_libraries(pray "-lc++experimental")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_link_libraries(pray "-lstdc++fs")
endif()

if (WITH_OMP)
  target_compile_options(pray PRIVATE ${OpenMP_CXX_FLAGS})
  if(NOT MSVC)
    target_link_libraries(pray ${OpenMP_CXX_FLAGS})
  endif()
elseif(NOT MSVC)
  target_compile_options(pray PRIVATE -Wno-error=unknown-pragmas)
endif()
