cmake_minimum_required (VERSION 4.0)

project ("kEmber")

# 调试信息：确认工具链是否生效
message(STATUS "Current Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")

# 要求 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 开启多处理器编译（MSVC）
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

# 新增：为 Release 配置启用 PDB 生成
if (MSVC)
  # /DEBUG: 生成 PDB 文件
  # /OPT:REF /OPT:ICF: 保持 Release 模式的优化（移除未引用函数/数据折叠）
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
endif()

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Polyscope
include(FetchContent)

FetchContent_Declare(
   polyscope
   GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
   GIT_TAG        master
)

FetchContent_MakeAvailable(polyscope)

# 查找第三方依赖
find_package(Eigen3 CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)

# 包含目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 收集源文件
file(GLOB_RECURSE SOURCES
  "src/*.cpp"
)

file(GLOB_RECURSE HEADERS
  "src/*.h"
  "include/*.h"
)

add_executable(kEmber ${SOURCES} ${HEADERS})

# 链接依赖
target_link_libraries(kEmber
  PUBLIC
    polyscope
  PRIVATE
    Eigen3::Eigen
    CGAL::CGAL
)
