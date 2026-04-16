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

option(KEMBER_ENABLE_EIGEN "Link Eigen3 from vcpkg" OFF)
option(KEMBER_ENABLE_CGAL "Link CGAL from vcpkg" OFF)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

add_executable(kEmber ${SOURCES} ${HEADERS})

target_include_directories(kEmber
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(kEmber PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(kEmber PRIVATE /MP)
  target_link_options(kEmber PRIVATE
    "$<$<CONFIG:Release>:/DEBUG>"
    "$<$<CONFIG:Release>:/OPT:REF>"
    "$<$<CONFIG:Release>:/OPT:ICF>"
  )
endif()

if(KEMBER_ENABLE_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)
  target_link_libraries(kEmber PRIVATE Eigen3::Eigen)
endif()

if(KEMBER_ENABLE_CGAL)
  find_package(CGAL CONFIG REQUIRED)
  target_link_libraries(kEmber PRIVATE CGAL::CGAL)
endif()
