cmake_minimum_required(VERSION 3.10)
project(StressCPU LANGUAGES CXX)

# C++14，关闭扩展
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 生成 compile_commands.json（便于 clangd/VSCode）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 单配置生成器（如 Unix Makefiles）下，默认 Release
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 头文件
include_directories(${PROJECT_SOURCE_DIR}/include)

# 线程库
find_package(Threads REQUIRED)

# 仅一个源文件生成可执行程序（名字：StressCPU）
add_executable(StressCPU
    src/StressCPU.cpp
)

target_link_libraries(StressCPU PRIVATE Threads::Threads)

if (MSVC)
  target_compile_options(StressCPU PRIVATE /W4 /permissive-)
else()
  target_compile_options(StressCPU PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()