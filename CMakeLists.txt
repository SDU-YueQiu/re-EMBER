# 构建 re-EMBER 库、命令行程序、可视化测试程序和自定义测试。
cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE
    "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE FILEPATH "Path to the vcpkg toolchain file"
  )
endif()

project(re-EMBER LANGUAGES CXX)

message(STATUS "Current Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  include(CheckCXXCompilerFlag)
  include(CheckCXXSourceRuns)
  add_compile_options(/utf-8)
endif()

option(REEMBER_ENABLE_EIGEN "Link Eigen3 from vcpkg" OFF)
option(REEMBER_ENABLE_CGAL "Link CGAL from vcpkg" OFF)
option(REEMBER_BUILD_VISUAL_TEST "Build the libigl-based visual-test target" ON)
option(REEMBER_ENABLE_TRACY "Enable optional Tracy profiler instrumentation" OFF)
option(REEMBER_ENABLE_TRACY_MATH "Enable optional low-level math256 Tracy instrumentation" OFF)
set(REEMBER_MSVC_ARCH "AUTO" CACHE STRING "MSVC /arch level: AUTO, OFF, SSE2, SSE4.2, AVX, AVX2, AVX512, AVX10.1, AVX10.2")

if(MSVC)
  set_property(CACHE REEMBER_MSVC_ARCH PROPERTY STRINGS
    AUTO
    OFF
    SSE2
    SSE4.2
    AVX
    AVX2
    AVX512
    AVX10.1
    AVX10.2
  )

  function(reember_get_msvc_arch_metadata level out_flag out_runtime_source)
    if(level STREQUAL "SSE2")
      set(_flag "/arch:SSE2")
      set(_runtime_source
        "#include <isa_availability.h>\n"
        "extern \"C\" int __cdecl __isa_available_init(void);\n"
        "extern \"C\" int __isa_available;\n"
        "int main() {\n"
        "  __isa_available_init();\n"
        "  return (__isa_available >= __ISA_AVAILABLE_SSE2) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "SSE4.2")
      set(_flag "/arch:SSE4.2")
      set(_runtime_source
        "#include <isa_availability.h>\n"
        "extern \"C\" int __cdecl __isa_available_init(void);\n"
        "extern \"C\" int __isa_available;\n"
        "int main() {\n"
        "  __isa_available_init();\n"
        "  return (__isa_available >= __ISA_AVAILABLE_SSE42) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "AVX")
      set(_flag "/arch:AVX")
      set(_runtime_source
        "#include <isa_availability.h>\n"
        "extern \"C\" int __cdecl __isa_available_init(void);\n"
        "extern \"C\" int __isa_available;\n"
        "int main() {\n"
        "  __isa_available_init();\n"
        "  return (__isa_available >= __ISA_AVAILABLE_AVX) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "AVX2")
      set(_flag "/arch:AVX2")
      set(_runtime_source
        "#include <isa_availability.h>\n"
        "extern \"C\" int __cdecl __isa_available_init(void);\n"
        "extern \"C\" int __isa_available;\n"
        "int main() {\n"
        "  __isa_available_init();\n"
        "  return (__isa_available >= __ISA_AVAILABLE_AVX2) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "AVX512")
      set(_flag "/arch:AVX512")
      set(_runtime_source
        "#include <isa_availability.h>\n"
        "extern \"C\" int __cdecl __isa_available_init(void);\n"
        "extern \"C\" int __isa_available;\n"
        "int main() {\n"
        "  __isa_available_init();\n"
        "  return (__isa_available >= __ISA_AVAILABLE_AVX512) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "AVX10.1")
      set(_flag "/arch:AVX10.1")
      set(_runtime_source
        "#include <immintrin.h>\n"
        "int main() {\n"
        "  return __check_isa_support(__IA_SUPPORT_AVX10, 1) ? 0 : 1;\n"
        "}\n"
      )
    elseif(level STREQUAL "AVX10.2")
      set(_flag "/arch:AVX10.2")
      set(_runtime_source
        "#include <immintrin.h>\n"
        "int main() {\n"
        "  return __check_isa_support(__IA_SUPPORT_AVX10_2, 2) ? 0 : 1;\n"
        "}\n"
      )
    else()
      message(FATAL_ERROR "Unsupported REEMBER_MSVC_ARCH level: ${level}")
    endif()

    set(${out_flag} "${_flag}" PARENT_SCOPE)
    set(${out_runtime_source} "${_runtime_source}" PARENT_SCOPE)
  endfunction()

  function(reember_check_msvc_arch_level level out_supported out_flag)
    reember_get_msvc_arch_metadata("${level}" _flag _runtime_source)

    string(MAKE_C_IDENTIFIER "${level}" _level_id)
    set(_flag_result_var "REEMBER_MSVC_ARCH_FLAG_${_level_id}")
    set(_runtime_result_var "REEMBER_MSVC_ARCH_RUNTIME_${_level_id}")

    set(_saved_required_quiet "${CMAKE_REQUIRED_QUIET}")
    set(CMAKE_REQUIRED_QUIET TRUE)
    check_cxx_compiler_flag("${_flag}" ${_flag_result_var})
    check_cxx_source_runs("${_runtime_source}" ${_runtime_result_var})
    set(CMAKE_REQUIRED_QUIET "${_saved_required_quiet}")

    if(${_flag_result_var} AND ${_runtime_result_var})
      set(${out_supported} TRUE PARENT_SCOPE)
    else()
      set(${out_supported} FALSE PARENT_SCOPE)
    endif()

    set(${out_flag} "${_flag}" PARENT_SCOPE)
  endfunction()

  function(reember_resolve_msvc_arch out_level out_flag)
    if(REEMBER_MSVC_ARCH STREQUAL "OFF")
      set(${out_level} "OFF" PARENT_SCOPE)
      set(${out_flag} "" PARENT_SCOPE)
      return()
    endif()

    if(REEMBER_MSVC_ARCH STREQUAL "AUTO")
      foreach(_level IN ITEMS AVX10.2 AVX10.1 AVX512 AVX2 AVX SSE4.2 SSE2)
        reember_check_msvc_arch_level("${_level}" _supported _flag)
        if(_supported)
          set(${out_level} "${_level}" PARENT_SCOPE)
          set(${out_flag} "${_flag}" PARENT_SCOPE)
          return()
        endif()
      endforeach()

      set(${out_level} "OFF" PARENT_SCOPE)
      set(${out_flag} "" PARENT_SCOPE)
      return()
    endif()

    reember_check_msvc_arch_level("${REEMBER_MSVC_ARCH}" _supported _flag)
    if(NOT _supported)
      message(FATAL_ERROR
        "Requested REEMBER_MSVC_ARCH=${REEMBER_MSVC_ARCH}, but either the compiler does not accept the flag or the current machine/runtime does not support it."
      )
    endif()

    set(${out_level} "${REEMBER_MSVC_ARCH}" PARENT_SCOPE)
    set(${out_flag} "${_flag}" PARENT_SCOPE)
  endfunction()

  function(reember_apply_msvc_arch target)
    if(REEMBER_RESOLVED_MSVC_ARCH_FLAG)
      target_compile_options(${target} PRIVATE "${REEMBER_RESOLVED_MSVC_ARCH_FLAG}")
    endif()
  endfunction()

  # /arch 是互斥单选项。默认 AUTO 会在当前构建机上从高到低检测：
  # 1. 编译器是否接受该 /arch 开关；
  # 2. 当前 CPU/OS 运行时是否真的支持该 ISA。
  reember_resolve_msvc_arch(REEMBER_RESOLVED_MSVC_ARCH_LEVEL REEMBER_RESOLVED_MSVC_ARCH_FLAG)
  if(REEMBER_RESOLVED_MSVC_ARCH_FLAG)
    message(STATUS
      "REEMBER_MSVC_ARCH=${REEMBER_MSVC_ARCH} -> ${REEMBER_RESOLVED_MSVC_ARCH_LEVEL} (${REEMBER_RESOLVED_MSVC_ARCH_FLAG})"
    )
  else()
    message(STATUS "REEMBER_MSVC_ARCH=${REEMBER_MSVC_ARCH} -> OFF")
  endif()
endif()

find_package(tinyobjloader CONFIG REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB CONFIG REQUIRED)

set(REEMBER_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

set(REEMBER_APP_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/main.cpp"
)

set(REEMBER_VISUAL_TEST_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/visual_test.cpp"
)

set(REEMBER_LIBRARY_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/WNV_tracing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/bsp.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/leaf_arrangement.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/bool_problem.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/leaf_classifier.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/subdivision_solver.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.cpp"
)

if(REEMBER_ENABLE_TRACY)
  list(APPEND REEMBER_LIBRARY_SOURCES
    "${REEMBER_THIRD_PARTY_DIR}/tracy/public/TracyClient.cpp"
  )
endif()

set(REEMBER_LIBRARY_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/WNV_tracing.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/bsp.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/leaf_arrangement.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidate_details.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidates.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/tracing_geometry.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/bool_problem.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/parallel_solve_context.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/perf_tracing.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/solver_shared.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/subdivision_solver.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/aabb.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/plane_geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/polygon_ops.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/math256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/winding_number_f.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/stl_reader/stl_reader.h"
)

set(REEMBER_TEST_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/bool_problem_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/io_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/math256_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/test_main.cpp"
)

set(REEMBER_TEST_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/bool_problem_tests.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/io_tests.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/math256_tests.h"
)

add_library(reember_lib ${REEMBER_LIBRARY_SOURCES} ${REEMBER_LIBRARY_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${REEMBER_LIBRARY_SOURCES} ${REEMBER_LIBRARY_HEADERS})

target_include_directories(reember_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${REEMBER_THIRD_PARTY_DIR}
    ${Boost_INCLUDE_DIRS}
)

target_compile_features(reember_lib PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(reember_lib PRIVATE /MP)
  target_compile_options(reember_lib PRIVATE
    "$<$<CONFIG:Release>:/Zi>"
  )
  reember_apply_msvc_arch(reember_lib)
endif()

target_link_libraries(reember_lib
  PRIVATE
    tinyobjloader::tinyobjloader_double
    TBB::tbb
)

if(REEMBER_ENABLE_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)
  target_link_libraries(reember_lib PUBLIC Eigen3::Eigen)
endif()

if(REEMBER_ENABLE_CGAL)
  find_package(CGAL CONFIG REQUIRED)
  target_link_libraries(reember_lib PUBLIC CGAL::CGAL)
endif()

if(REEMBER_ENABLE_TRACY)
  find_package(Threads REQUIRED)
  target_compile_definitions(reember_lib PUBLIC REEMBER_ENABLE_TRACY=1 TRACY_ENABLE=1)
  if(REEMBER_ENABLE_TRACY_MATH)
    target_compile_definitions(reember_lib PUBLIC REEMBER_ENABLE_TRACY_MATH=1)
  endif()
  target_compile_definitions(reember_lib PRIVATE TRACY_ONLY_IPV4 TRACY_NO_BROADCAST)
  target_include_directories(reember_lib PUBLIC "${REEMBER_THIRD_PARTY_DIR}/tracy/public")
  target_link_libraries(reember_lib PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
  if(WIN32)
    target_link_libraries(reember_lib PRIVATE ws2_32 dbghelp)
  endif()
endif()

add_executable(re-EMBER ${REEMBER_APP_SOURCE})
target_compile_definitions(re-EMBER PRIVATE TINYOBJLOADER_USE_DOUBLE)
target_link_libraries(re-EMBER PRIVATE reember_lib)

if(MSVC)
  target_compile_options(re-EMBER PRIVATE
    "$<$<CONFIG:Release>:/Zi>"
  )
  target_link_options(re-EMBER PRIVATE
    "$<$<CONFIG:Release>:/DEBUG:FULL>"
    "$<$<CONFIG:Release>:/OPT:REF>"
    "$<$<CONFIG:Release>:/OPT:ICF>"
    "$<$<CONFIG:RelWithDebInfo>:/DEBUG:FULL>"
  )
  reember_apply_msvc_arch(re-EMBER)
endif()

add_executable(re-EMBER_tests ${REEMBER_TEST_SOURCES} ${REEMBER_TEST_HEADERS})
target_compile_definitions(re-EMBER_tests PRIVATE TINYOBJLOADER_USE_DOUBLE)
target_link_libraries(re-EMBER_tests PRIVATE reember_lib)

if(MSVC)
  reember_apply_msvc_arch(re-EMBER_tests)
endif()

if(REEMBER_BUILD_VISUAL_TEST)
  find_package(Eigen3 CONFIG REQUIRED)
  find_package(CGAL CONFIG REQUIRED)
  find_package(libigl CONFIG REQUIRED)

  add_executable(visual-test
    ${REEMBER_VISUAL_TEST_SOURCE}
  )
  target_compile_definitions(visual-test PRIVATE REEMBER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(visual-test
    PRIVATE
      reember_lib
      Eigen3::Eigen
      CGAL::CGAL
      igl::igl_glfw
      igl::igl_imgui
  )
  target_precompile_headers(visual-test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src/application/visual_test_pch.h"
  )
  if(MSVC)
    reember_apply_msvc_arch(visual-test)
  endif()
endif()

enable_testing()
add_test(NAME re-EMBER_tests COMMAND re-EMBER_tests)
