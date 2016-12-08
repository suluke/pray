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

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/ext"
  )

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /fp:fast")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -Wall -Werror -march=native")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
endif()

add_subdirectory(src)

configure_file(
  "${PROJECT_SOURCE_DIR}/include/pray/Config.h.in"
  "${PROJECT_BINARY_DIR}/Config.h.in"
  )
file(GENERATE
  OUTPUT "${PROJECT_BINARY_DIR}/include/pray/Config.h"
  INPUT "${PROJECT_BINARY_DIR}/Config.h.in"
  )
