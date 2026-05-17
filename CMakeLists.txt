# 构建 re-EMBER 库、命令行程序、可视化测试程序和自定义测试。
cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE
    "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE FILEPATH "Path to the vcpkg toolchain file"
  )
endif()

if(WIN32 AND NOT DEFINED CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
  find_program(REEMBER_DEFAULT_CLANG_CL
    NAMES clang-cl
    HINTS
      "$ENV{USERPROFILE}/scoop/apps/llvm/current/bin"
      "$ENV{ProgramFiles}/LLVM/bin"
  )
  if(REEMBER_DEFAULT_CLANG_CL)
    set(CMAKE_CXX_COMPILER
      "${REEMBER_DEFAULT_CLANG_CL}"
      CACHE FILEPATH "Default C++ compiler for re-EMBER"
      FORCE
    )
  endif()
endif()

project(re-EMBER LANGUAGES CXX)

message(STATUS "Current Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "Current C++ compiler: ${CMAKE_CXX_COMPILER}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(REEMBER_ENABLE_EIGEN "Link Eigen3 from vcpkg" OFF)
option(REEMBER_ENABLE_CGAL "Link CGAL from vcpkg" OFF)
option(REEMBER_BUILD_VISUAL_TEST "Build the libigl-based visual-test target" ON)
option(REEMBER_BUILD_VERIFY "Build the CGAL Nef oracle verifier target" ON)
option(REEMBER_ENABLE_TRACY "Enable optional Tracy profiler instrumentation" OFF)
option(REEMBER_ENABLE_TRACY_MATH "Enable optional low-level math256 Tracy instrumentation" OFF)

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

set(REEMBER_VERIFY_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/verify.cpp"
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

if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options(reember_lib PRIVATE /MP)
endif()

if(MSVC)
  target_compile_options(reember_lib PRIVATE
    "$<$<CONFIG:Release>:/Zi>"
  )
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
endif()

add_executable(re-EMBER_tests ${REEMBER_TEST_SOURCES} ${REEMBER_TEST_HEADERS})
target_compile_definitions(re-EMBER_tests PRIVATE TINYOBJLOADER_USE_DOUBLE)
target_link_libraries(re-EMBER_tests PRIVATE reember_lib)
add_dependencies(re-EMBER_tests re-EMBER)

if(REEMBER_BUILD_VERIFY)
  find_package(CGAL CONFIG REQUIRED)

  add_executable(re-EMBER_verify ${REEMBER_VERIFY_SOURCE})
  target_compile_definitions(re-EMBER_verify PRIVATE TINYOBJLOADER_USE_DOUBLE)
  target_link_libraries(re-EMBER_verify PRIVATE reember_lib CGAL::CGAL)
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
endif()

enable_testing()
add_test(NAME re-EMBER_tests COMMAND re-EMBER_tests)

include(ProcessorCount)
ProcessorCount(REEMBER_HOST_PROCESSOR_COUNT)
if(NOT REEMBER_HOST_PROCESSOR_COUNT)
  set(REEMBER_HOST_PROCESSOR_COUNT 1)
endif()
set(REEMBER_CTEST_THREADS "${REEMBER_HOST_PROCESSOR_COUNT}" CACHE STRING "Thread count used by re-EMBER CLI CTest workloads.")
message(STATUS "REEMBER_CTEST_THREADS=${REEMBER_CTEST_THREADS}")
find_program(REEMBER_POWERSHELL_EXECUTABLE NAMES powershell REQUIRED)

set(REEMBER_PAPER_EXPERIMENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/paper_experiments")

# 论文 10% 实验 small oracle-success 样本，统一走批量 profiling 脚本并输出汇总报告。
add_test(
  NAME re-EMBER_paper_small_batch
  COMMAND "${REEMBER_POWERSHELL_EXECUTABLE}"
    -ExecutionPolicy Bypass
    -File "${CMAKE_CURRENT_SOURCE_DIR}/tools/profile-re-ember.ps1"
    -ExecutablePath "$<TARGET_FILE:re-EMBER>"
    -InputRoot "${REEMBER_PAPER_EXPERIMENT_DIR}/inputs/small"
    -Configuration "$<CONFIG>"
    -Op difference
    -NoTracy
    -Iterations 1
    -LeafThreshold 25
    -Threads "${REEMBER_CTEST_THREADS}"
)
set_tests_properties(re-EMBER_paper_small_batch PROPERTIES
  LABELS "paper_experiment;cli"
  TIMEOUT 120
  RUN_SERIAL TRUE
)
