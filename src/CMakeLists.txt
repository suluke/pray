add_executable(pray
  ext.cpp
  main.cpp
  scene.cpp
  scene_io.cpp
  )

target_compile_options(pray PUBLIC ${OpenMP_CXX_FLAGS})
target_link_libraries(pray ${OpenMP_CXX_FLAGS})
