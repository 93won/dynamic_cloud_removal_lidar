cmake_minimum_required(VERSION 3.16)
project(dynablox_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)

# Voxblox (dynablox/release branch)
set(VOXBLOX_DIR ${CMAKE_SOURCE_DIR}/thirdparty/voxblox)

# Source files  
set(DYNABLOX_SOURCES
    src/ever_free_integrator.cpp
    src/clustering.cpp
    src/motion_detector.cpp
)

# Header files
set(DYNABLOX_HEADERS
    include/dynablox/types.h
    include/dynablox/config.h
    include/dynablox/neighborhood_search.h
    include/dynablox/ever_free_integrator.h
    include/dynablox/clustering.h
    include/dynablox/motion_detector.h
)

# Create library
add_library(${PROJECT_NAME} ${DYNABLOX_SOURCES} ${DYNABLOX_HEADERS})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${VOXBLOX_DIR}/voxblox/include
        ${GLOG_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        yaml-cpp
        ${GLOG_LIBRARIES}
        voxblox
)

# Alias for easier use
add_library(dynablox::dynablox ALIAS ${PROJECT_NAME})
