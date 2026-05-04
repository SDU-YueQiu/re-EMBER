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

find_package(spdlog CONFIG REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)

set(KEMBER_APP_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/main.cpp"
)

set(KEMBER_VISUAL_TEST_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/visual_test.cpp"
)

file(GLOB_RECURSE KEMBER_LIBRARY_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/*.cpp"
)

file(GLOB_RECURSE KEMBER_LIBRARY_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/io/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/math/*.h"
)

file(GLOB_RECURSE KEMBER_TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/*.cpp"
)

file(GLOB_RECURSE KEMBER_TEST_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/*.h"
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
  target_precompile_headers(visual-test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src/application/visual_test_pch.h"
  )
endif()

enable_testing()
add_test(NAME kEmber_tests COMMAND kEmber_tests)
