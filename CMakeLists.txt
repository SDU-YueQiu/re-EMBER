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

file(GLOB_RECURSE KEMBER_ALL_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE KEMBER_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

file(GLOB_RECURSE KEMBER_TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/*.cpp"
)

set(KEMBER_APP_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/main.cpp"
)

set(KEMBER_LIBRARY_SOURCES ${KEMBER_ALL_SOURCES})
list(REMOVE_ITEM KEMBER_LIBRARY_SOURCES ${KEMBER_APP_SOURCE})
list(REMOVE_ITEM KEMBER_LIBRARY_SOURCES ${KEMBER_TEST_SOURCES})

add_library(kember_lib ${KEMBER_LIBRARY_SOURCES} ${KEMBER_HEADERS})

target_include_directories(kember_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(kember_lib PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(kember_lib PRIVATE /MP)
endif()

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

add_executable(kEmber_tests ${KEMBER_TEST_SOURCES})
target_link_libraries(kEmber_tests PRIVATE kember_lib)

enable_testing()
add_test(NAME kEmber_tests COMMAND kEmber_tests)

