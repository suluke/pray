add_executable(pray_debug_tool
  main.cpp
  ../src/ext.cpp
  ../src/scene.cpp
  )

target_compile_definitions(pray_debug_tool PRIVATE DEBUG_TOOL)

target_include_directories(pray_debug_tool PRIVATE ../src)
set_target_properties(pray_debug_tool PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	target_link_libraries(pray_debug_tool "-lc++experimental")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_link_libraries(pray_debug_tool "-lstdc++fs")
endif()
target_include_directories(pray_debug_tool PRIVATE ${GLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
target_link_libraries(pray_debug_tool ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY})
if (WITH_OMP)
  target_compile_options(pray_debug_tool PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(pray_debug_tool ${OpenMP_CXX_FLAGS})
elseif(NOT MSVC)
  target_compile_options(pray_debug_tool PRIVATE -Wno-error=unknown-pragmas)
endif()
