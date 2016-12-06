add_executable(pray
  debug.hpp
  ext.cpp
  ext.hpp
  main.cpp
  math.hpp
  scene.cpp
  scene.hpp
  cpu_tracer.hpp
  cpu_tracer.cpp
  )

set_target_properties(pray PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_link_libraries(pray "-lstdc++fs")
if (WITH_OMP)
  target_compile_options(pray PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(pray ${OpenMP_CXX_FLAGS})
else()
  target_compile_options(pray PRIVATE -Wno-error=unknown-pragmas)
endif()
