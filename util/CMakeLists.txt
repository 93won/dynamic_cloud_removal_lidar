cmake_minimum_required(VERSION 3.16)
project(util)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# Header-only nanoflann
set(NANOFLANN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/nanoflann)

# Source files
set(UTIL_SOURCES
    src/point_cloud.cpp
    src/io.cpp
)

# Header files
set(UTIL_HEADERS
    include/util/point_cloud.h
    include/util/kdtree.h
    include/util/io.h
)

# Create library
add_library(${PROJECT_NAME} ${UTIL_SOURCES} ${UTIL_HEADERS})

target_include_directories(${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${NANOFLANN_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
)

# Alias for easier use in parent CMake
add_library(util::util ALIAS ${PROJECT_NAME})
