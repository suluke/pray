cmake_minimum_required(VERSION 3.0.2)
project(pray CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

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
option(WITH_SSE_PT "Use SSE Rays in path tracing mode" ON)

set(SAMPLER "standard" CACHE STRING "The sampler to be used")
set(SAMPLER_NAMES "standard" "interpolating" "adaptive")
set_property(CACHE SAMPLER PROPERTY STRINGS ${SAMPLER_NAMES})
list(FIND SAMPLER_NAMES ${SAMPLER} SAMPLER_INDEX)
if(${SAMPLER_INDEX} EQUAL -1)
 message(FATAL_ERROR "Sampler ${SAMPLER} not supported. Valid values are ${SAMPLER_NAMES}")
endif()

set(ACCELERATOR "Bih" CACHE STRING "The acceleration structure to be used")
set(ACCELERATOR_NAMES "DummyAcceleration" "Bih" "KdTree")
set_property(CACHE ACCELERATOR PROPERTY STRINGS ${ACCELERATOR_NAMES})
list(FIND ACCELERATOR_NAMES ${ACCELERATOR} ACCELERATOR_INDEX)
if(${ACCELERATOR_INDEX} EQUAL -1)
 message(FATAL_ERROR "Acceleration structure ${ACCELERATOR} not supported. Valid values are ${ACCELERATOR_NAMES}")
endif()

option(WITH_BIH_PARALLEL_BUILD "Build with BIH parallelized build" OFF)
option(WITH_TIMING "Build with logging of processing time enabled" ON)
option(WITH_CONFDUMP "Build with logging all activated features on startup" ON)
option(WITH_PROGRESS "Build with periodically printing of progress information" OFF)
option(DISABLE_RENDERING "Disable rendering. Useful for profiling preprocessing" OFF)
option(DISABLE_OUTPUT "Disable writing out the image. Useful for profiling" OFF)

option(WITH_DEBUG_TOOL "Build the debug tool" OFF)
if (WITH_DEBUG_TOOL)
  find_package(OpenGL)
  find_package(GLUT)
endif()

macro(print_opt OPT)
  if(${OPT})
    message(STATUS "${OPT} is ${${OPT}}")
  else()
    message(STATUS "${OPT} is OFF")
  endif()
endmacro()

message(STATUS "")
message(STATUS "##### CONFIG #####")
print_opt(WITH_OMP)
print_opt(WITH_CUDA)
print_opt(WITH_SSE)
print_opt(WITH_SSE_PT)
print_opt(ACCELERATOR)
print_opt(SAMPLER)
print_opt(WITH_BIH_PARALLEL_BUILD)
print_opt(WITH_TIMING)
print_opt(WITH_DEBUG_TOOL)
print_opt(WITH_CONFDUMP)
print_opt(WITH_PROGRESS)
print_opt(DISABLE_RENDERING)
print_opt(DISABLE_OUTPUT)
message(STATUS "##################\n")

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_BINARY_DIR}/include"
  "${PROJECT_SOURCE_DIR}/ext"
  )

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS /arch:SSE2 /D__SSE2__")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /fp:fast")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -Og")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -Wall -Werror")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
endif()

set(CUDA_NVCC_FLAGS
  ${CUDA_NVCC_FLAGS}
  ${CUDA_NVCC_CCBIN}
  -std=c++11
  --expt-relaxed-constexpr
  -libcudart
  -gencode arch=compute_35,code=compute_35 # for pc189
  -gencode arch=compute_20,code=sm_20 # for pc205
  
  # to avoid error "identifier __builtin_ia32_mwaitx is undefined
  # https://github.com/tensorflow/tensorflow/issues/1066
  -D_MWAITXINTRIN_H_INCLUDED
  -D_FORCE_INLINES
  )
set(CUDA_PROPAGATE_HOST_FLAGS off)

add_subdirectory(src)

if(WITH_DEBUG_TOOL)
  add_subdirectory(tools/debug_tool)
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/include/pray/Config.h.in"
  "${PROJECT_BINARY_DIR}/Config.h.in"
  )
file(GENERATE
  OUTPUT "${PROJECT_BINARY_DIR}/include/pray/Config.h"
  INPUT "${PROJECT_BINARY_DIR}/Config.h.in"
  )
