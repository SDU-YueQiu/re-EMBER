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
  add_compile_options(/utf-8)
endif()

option(REEMBER_ENABLE_EIGEN "Link Eigen3 from vcpkg" OFF)
option(REEMBER_ENABLE_CGAL "Link CGAL from vcpkg" OFF)
option(REEMBER_BUILD_VISUAL_TEST "Build the libigl-based visual-test target" ON)

find_package(spdlog CONFIG REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)

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
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/logging.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/subdivision_solver.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.cpp"
)

set(REEMBER_LIBRARY_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_fixdiv.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_int.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_io.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_gcc.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_long.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_msvc.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_uint.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/WNV_tracing.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/bsp.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/leaf_arrangement.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidate_details.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidates.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/tracing_geometry.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/bool_problem.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/logging.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/subdivision_solver.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/aabb.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/plane_geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/polygon_ops.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/math256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/winding_number_f.h"
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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(reember_lib PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(reember_lib PRIVATE /MP)
  target_compile_options(reember_lib PRIVATE
    "$<$<CONFIG:Release>:/Zi>"
  )
endif()

target_link_libraries(reember_lib
  PRIVATE
    spdlog::spdlog
    tinyobjloader::tinyobjloader_double
)

if(REEMBER_ENABLE_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)
  target_link_libraries(reember_lib PUBLIC Eigen3::Eigen)
endif()

if(REEMBER_ENABLE_CGAL)
  find_package(CGAL CONFIG REQUIRED)
  target_link_libraries(reember_lib PUBLIC CGAL::CGAL)
endif()

add_executable(re-EMBER ${REEMBER_APP_SOURCE})
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
target_link_libraries(re-EMBER_tests PRIVATE reember_lib)

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
