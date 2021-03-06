if (CMAKE_SYSTEM_NAME MATCHES "Linux")
  exec_program(cat ARGS "/proc/cpuinfo" OUTPUT_VARIABLE CPUINFO)

  string(REGEX REPLACE "^.* (pni) .*$" "\\1" _SSE_THERE ${CPUINFO})
  string(COMPARE EQUAL "pni" "${_SSE_THERE}" _SSE3_TRUE)
  if (_SSE3_TRUE)
    set(SSE3_FOUND true CACHE BOOL "SSE3 available on host")
  else()
    set(SSE3_FOUND false CACHE BOOL "SSE3 available on host")
  endif()
  mark_as_advanced(SSE3_FOUND)

  string(REGEX REPLACE "^.* (avx) .*$" "\\1" _SSE_THERE ${CPUINFO})
  string(COMPARE EQUAL "avx" "${_SSE_THERE}" _AVX_TRUE)
  if (_AVX_TRUE)
    set(AVX_FOUND true CACHE BOOL "AVX available")
  else()
    set(AVX_FOUND false CACHE BOOL "AVX available")
  endif()
  mark_as_advanced(AVX_FOUND)

  if (SSE3_FOUND OR AVX_FOUND)
    set(SIMD_FOUND true CACHE BOOL "SSE available")
  else()
    set(SIMD_FOUND false CACHE BOOL "SSE available")
  endif()
  mark_as_advanced(SIMD_FOUND)
endif()
