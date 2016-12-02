cmake_minimum_required(VERSION 3.0.2)
project(pray CXX C)

find_package(OpenMP REQUIRED)

option(WITH_CUDA "Build with CUDA support" ON)
find_package(CUDA)
if (NOT CUDA_FOUND OR NOT CUDA_CUDA_LIBRARY)
  set(WITH_CUDA OFF CACHE BOOL "Build with CUDA support" FORCE)
endif()

message(STATUS ${OpenMP_CXX_FLAGS})

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/ext"
  )

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /fp:fast")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -Wall -Werror ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
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
