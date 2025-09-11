#=============================================================================
# CMakeLists.txt
#=============================================================================
cmake_minimum_required(VERSION 3.5)
project(ThreadCtl CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 强制 out-of-source 构建
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "请在独立的 build/ 目录下执行 cmake ..")
endif()

# 库
add_library(threadctl STATIC
    ${PROJECT_SOURCE_DIR}/src/ThreadCtl.cpp
)

target_include_directories(threadctl PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_executable(rms
    ${PROJECT_SOURCE_DIR}/src/RMS.cpp
)
target_link_libraries(rms threadctl)

add_executable(rms-dependency
    ${PROJECT_SOURCE_DIR}/src/RMS-Dependency.cpp
)
target_link_libraries(rms-dependency threadctl)

add_executable(edf
    ${PROJECT_SOURCE_DIR}/src/EDF.cpp
)
target_link_libraries(edf threadctl)

add_executable(cfs
    ${PROJECT_SOURCE_DIR}/src/CFS.cpp
)
target_link_libraries(cfs threadctl)
