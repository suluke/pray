cmake_minimum_required(VERSION 3.0.2)
project(pray CXX C)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(WITH_OMP "Build with OpenMP support" ON)
find_package(OpenMP)
if (NOT OpenMP_FOUND)
  set(WITH_OMP OFF CACHE BOOL "Build with OpenMP support" FORCE)
endif()

option(WITH_CUDA "Build with CUDA support" ON)
find_package(CUDA)
if (NOT CUDA_FOUND OR NOT CUDA_CUDA_LIBRARY)
  set(WITH_CUDA OFF CACHE BOOL "Build with CUDA support" FORCE)
endif()

option(WITH_SSE "Build with SIMD (SSE) support" ON)
find_package(SIMD)
if (NOT SIMD_FOUND)
  set(WITH_SSE OFF CACHE BOOL "Build with SIMD (SSE) support" FORCE)
endif()

option(WITH_SUBSAMPLING "Build with adaptive subsampling enabled" OFF)
option(WITH_BIH "Build with BIH acceleration structure" OFF)
option(WITH_TIMING "Build with logging of processing time enabled" OFF)

option(WITH_DEBUG_TOOL "Build the debug tool" OFF)
if  (WITH_DEBUG_TOOL)
  find_package(OpenGL)
  find_package(GLUT)
endif()

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_BINARY_DIR}/include"
  "${PROJECT_SOURCE_DIR}/ext"
  )

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS /arch:SSE2 /D__SSE2__")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /fp:fast")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Werror -march=native -Og")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
endif()

add_subdirectory(src)

if(WITH_DEBUG_TOOL)
  add_subdirectory(debug_tool)
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/include/pray/Config.h.in"
  "${PROJECT_BINARY_DIR}/Config.h.in"
  )
file(GENERATE
  OUTPUT "${PROJECT_BINARY_DIR}/include/pray/Config.h"
  INPUT "${PROJECT_BINARY_DIR}/Config.h.in"
  )
