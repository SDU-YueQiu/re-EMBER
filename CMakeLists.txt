cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE
    "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE FILEPATH "Path to the vcpkg toolchain file"
  )
endif()

project(kEmber LANGUAGES CXX)

message(STATUS "Current Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(KEMBER_ENABLE_EIGEN "Link Eigen3 from vcpkg" OFF)
option(KEMBER_ENABLE_CGAL "Link CGAL from vcpkg" OFF)
option(KEMBER_ENABLE_VISUAL_TEST "Build the visual-test application target" OFF)

set(KEMBER_REQUIRED_VCPKG_COMMAND
  "vcpkg install spdlog tinyobjloader[double] --triplet x64-windows"
)

find_package(spdlog CONFIG QUIET)
if(NOT spdlog_FOUND OR NOT TARGET spdlog::spdlog)
  message(FATAL_ERROR
    "Missing required dependency spdlog. Install dependencies with: "
    "${KEMBER_REQUIRED_VCPKG_COMMAND}"
  )
endif()

find_package(tinyobjloader CONFIG QUIET)
if(NOT tinyobjloader_FOUND OR NOT TARGET tinyobjloader::tinyobjloader_double)
  message(FATAL_ERROR
    "Missing required dependency tinyobjloader double target. Install dependencies with: "
    "${KEMBER_REQUIRED_VCPKG_COMMAND}"
  )
endif()

set(KEMBER_APP_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/main.cpp"
)

set(KEMBER_VISUAL_TEST_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/visual_test.cpp"
)

set(KEMBER_MATH_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/math256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/winding_number_f.h"
)

set(KEMBER_GEOMETRY_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/aabb.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/polygon_ops.cpp"
)

set(KEMBER_GEOMETRY_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/aabb.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/clipping.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/plane_geometry256.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/polygon_ops.h"
)

set(KEMBER_ALGORITHM_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/WNV_tracing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/bsp.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/leaf_arrangement.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidates.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/tracing_geometry.cpp"
)

set(KEMBER_ALGORITHM_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/WNV_tracing.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/bsp.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/leaf_arrangement.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/path_candidates.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/tracing_geometry.h"
)

set(KEMBER_CORE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/bool_problem.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/leaf_classifier.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/logging.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/subdivision_solver.cpp"
)

set(KEMBER_CORE_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/bool_problem.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/logging.h"
)

set(KEMBER_IO_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.cpp"
)

set(KEMBER_IO_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/io.h"
)

set(KEMBER_EXTERNAL_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_fixdiv.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_int.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_io.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_gcc.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_long.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_math_msvc.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/slimcpplib/long_uint.h"
)

set(KEMBER_LIBRARY_SOURCES
  ${KEMBER_GEOMETRY_SOURCES}
  ${KEMBER_ALGORITHM_SOURCES}
  ${KEMBER_CORE_SOURCES}
  ${KEMBER_IO_SOURCES}
)

set(KEMBER_LIBRARY_HEADERS
  ${KEMBER_EXTERNAL_HEADERS}
  ${KEMBER_MATH_HEADERS}
  ${KEMBER_GEOMETRY_HEADERS}
  ${KEMBER_ALGORITHM_HEADERS}
  ${KEMBER_CORE_HEADERS}
  ${KEMBER_IO_HEADERS}
)

set(KEMBER_TEST_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/bool_problem_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/io_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/math256_tests.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/test_main.cpp"
)

set(KEMBER_TEST_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/bool_problem_tests.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/io_tests.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/math256_tests.h"
)

add_library(kember_lib ${KEMBER_LIBRARY_SOURCES} ${KEMBER_LIBRARY_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${KEMBER_LIBRARY_SOURCES} ${KEMBER_LIBRARY_HEADERS})

target_include_directories(kember_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(kember_lib PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(kember_lib PRIVATE /MP)
endif()

target_link_libraries(kember_lib
  PRIVATE
    spdlog::spdlog
    tinyobjloader::tinyobjloader_double
)

if(KEMBER_ENABLE_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)
  target_link_libraries(kember_lib PUBLIC Eigen3::Eigen)
endif()

if(KEMBER_ENABLE_CGAL)
  find_package(CGAL CONFIG REQUIRED)
  target_link_libraries(kember_lib PUBLIC CGAL::CGAL)
endif()

add_executable(kEmber ${KEMBER_APP_SOURCE})
target_link_libraries(kEmber PRIVATE kember_lib)

if(MSVC)
  target_link_options(kEmber PRIVATE
    "$<$<CONFIG:Release>:/DEBUG>"
    "$<$<CONFIG:Release>:/OPT:REF>"
    "$<$<CONFIG:Release>:/OPT:ICF>"
  )
endif()

add_executable(kEmber_tests ${KEMBER_TEST_SOURCES} ${KEMBER_TEST_HEADERS})
target_link_libraries(kEmber_tests PRIVATE kember_lib)

if(KEMBER_ENABLE_VISUAL_TEST)
  find_package(Eigen3 CONFIG REQUIRED)
  find_package(CGAL CONFIG REQUIRED)
  find_package(libigl CONFIG REQUIRED)

  add_executable(visual-test
    ${KEMBER_VISUAL_TEST_SOURCE}
  )
  target_compile_definitions(visual-test PRIVATE KEMBER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(visual-test
    PRIVATE
      kember_lib
      Eigen3::Eigen
      CGAL::CGAL
      igl::igl_glfw
      igl::igl_imgui
  )
endif()

enable_testing()
add_test(NAME kEmber_tests COMMAND kEmber_tests)

