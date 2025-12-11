cmake_minimum_required(VERSION 3.16)
project(evaluation_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)

# Pangolin (built from thirdparty)
set(Pangolin_DIR ${CMAKE_SOURCE_DIR}/thirdparty/Pangolin/build)
find_package(Pangolin QUIET)

# Option to enable Pangolin viewer
option(USE_PANGOLIN "Build with Pangolin visualization" ON)

# Source files
set(EVAL_SOURCES
    src/main.cpp
)

if(USE_PANGOLIN AND Pangolin_FOUND)
    list(APPEND EVAL_SOURCES src/pangolin_viewer.cpp)
    add_definitions(-DUSE_PANGOLIN)
endif()

# Create executable
add_executable(${PROJECT_NAME} ${EVAL_SOURCES})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Eigen3::Eigen
        yaml-cpp
        util
        dynablox_standalone
)

if(USE_PANGOLIN AND Pangolin_FOUND)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ${Pangolin_LIBRARIES}
            OpenGL::GL
    )
    target_include_directories(${PROJECT_NAME}
        PRIVATE
            ${Pangolin_INCLUDE_DIRS}
    )
endif()

# Alias
add_executable(eval::tool ALIAS ${PROJECT_NAME})

# GT Viewer executable (simple viewer for ground truth visualization)
if(USE_PANGOLIN AND Pangolin_FOUND)
    add_executable(gt_viewer src/gt_viewer.cpp)
    
    target_include_directories(gt_viewer
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${Pangolin_INCLUDE_DIRS}
    )
    
    target_link_libraries(gt_viewer
        PRIVATE
            Eigen3::Eigen
            ${Pangolin_LIBRARIES}
            OpenGL::GL
            stdc++fs
    )
endif()

# Dynablox Runner executable (run Dynablox detector on point cloud sequence)
if(USE_PANGOLIN AND Pangolin_FOUND)
    add_executable(dynablox_runner src/dynablox_runner.cpp)
    
    target_include_directories(dynablox_runner
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${Pangolin_INCLUDE_DIRS}
            ${CMAKE_SOURCE_DIR}/method/dynablox_standalone/include
            ${CMAKE_SOURCE_DIR}/util/include
    )
    
    target_link_libraries(dynablox_runner
        PRIVATE
            Eigen3::Eigen
            yaml-cpp
            ${Pangolin_LIBRARIES}
            OpenGL::GL
            stdc++fs
            util
            dynablox_standalone
    )
endif()
