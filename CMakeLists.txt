cmake_minimum_required(VERSION 3.16)
project(dynamic_cloud_removal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)

# Optional: yaml-cpp for configuration
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    message(STATUS "Found yaml-cpp")
else()
    message(STATUS "yaml-cpp not found, will use default configs")
endif()

# Thirdparty directory
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)

# Include nanoflann (header-only)
include_directories(${THIRDPARTY_DIR}/nanoflann)

# Build Pangolin
set(BUILD_PANGOLIN_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PANGOLIN_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_PANGOLIN_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${THIRDPARTY_DIR}/pangolin ${CMAKE_BINARY_DIR}/pangolin EXCLUDE_FROM_ALL)

# Build Voxblox (standalone, no ROS)
add_subdirectory(${THIRDPARTY_DIR}/voxblox_standalone ${CMAKE_BINARY_DIR}/voxblox)

# Build util library
add_subdirectory(util)

# Build method libraries
add_subdirectory(method/dynablox_standalone)
# add_subdirectory(method/dufomap_standalone)  # Future

# Build evaluation tool
add_subdirectory(evaluation_tool)

message(STATUS "")
message(STATUS "=== Configuration Summary ===")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "=============================")
