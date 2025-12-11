# Voxblox Standalone Build (No ROS)
# This builds the voxblox library without ROS dependencies

cmake_minimum_required(VERSION 3.16)
project(voxblox)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)

# glog and gflags
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GLOG libglog)
    pkg_check_modules(GFLAGS gflags)
endif()

# Voxblox source directory
set(VOXBLOX_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../voxblox/voxblox)
set(VOXBLOX_PROTO_DIR ${VOXBLOX_SRC_DIR}/proto)
set(KINDR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../kindr/include)
set(MINKINDR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../minkindr/minkindr/include)

# Generate protobuf files
set(PROTO_FILES
    ${VOXBLOX_PROTO_DIR}/voxblox/Block.proto
    ${VOXBLOX_PROTO_DIR}/voxblox/Layer.proto
)

# Output directory for generated files
set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR}/voxblox)

# Generate C++ from proto files
foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(PROTO_SRC ${PROTO_OUTPUT_DIR}/voxblox/${PROTO_NAME}.pb.cc)
    set(PROTO_HDR ${PROTO_OUTPUT_DIR}/voxblox/${PROTO_NAME}.pb.h)
    
    add_custom_command(
        OUTPUT ${PROTO_SRC} ${PROTO_HDR}
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            --proto_path=${VOXBLOX_PROTO_DIR}
            --cpp_out=${PROTO_OUTPUT_DIR}
            ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating protobuf for ${PROTO_NAME}"
    )
    
    list(APPEND PROTO_SRCS ${PROTO_SRC})
    list(APPEND PROTO_HDRS ${PROTO_HDR})
endforeach()

# Include directories
include_directories(
    ${VOXBLOX_SRC_DIR}/include
    ${PROTO_OUTPUT_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${KINDR_INCLUDE_DIR}
    ${MINKINDR_INCLUDE_DIR}
)

# Source files
set(VOXBLOX_SRCS
    ${VOXBLOX_SRC_DIR}/src/core/block.cc
    ${VOXBLOX_SRC_DIR}/src/core/esdf_map.cc
    ${VOXBLOX_SRC_DIR}/src/core/tsdf_map.cc
    ${VOXBLOX_SRC_DIR}/src/integrator/esdf_integrator.cc
    ${VOXBLOX_SRC_DIR}/src/integrator/esdf_occ_integrator.cc
    ${VOXBLOX_SRC_DIR}/src/integrator/integrator_utils.cc
    ${VOXBLOX_SRC_DIR}/src/integrator/intensity_integrator.cc
    ${VOXBLOX_SRC_DIR}/src/integrator/tsdf_integrator.cc
    ${VOXBLOX_SRC_DIR}/src/io/mesh_ply.cc
    ${VOXBLOX_SRC_DIR}/src/io/sdf_ply.cc
    ${VOXBLOX_SRC_DIR}/src/mesh/marching_cubes.cc
    ${VOXBLOX_SRC_DIR}/src/simulation/objects.cc
    ${VOXBLOX_SRC_DIR}/src/simulation/simulation_world.cc
    ${VOXBLOX_SRC_DIR}/src/utils/camera_model.cc
    ${VOXBLOX_SRC_DIR}/src/utils/evaluation_utils.cc
    ${VOXBLOX_SRC_DIR}/src/utils/layer_utils.cc
    ${VOXBLOX_SRC_DIR}/src/utils/neighbor_tools.cc
    ${VOXBLOX_SRC_DIR}/src/utils/timing.cc
    ${VOXBLOX_SRC_DIR}/src/utils/voxel_utils.cc
    ${VOXBLOX_SRC_DIR}/src/utils/protobuf_utils.cc
)

# Create library with protobuf sources
add_library(voxblox ${VOXBLOX_SRCS} ${PROTO_SRCS})

target_include_directories(voxblox PUBLIC
    ${VOXBLOX_SRC_DIR}/include
    ${PROTO_OUTPUT_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${KINDR_INCLUDE_DIR}
    ${MINKINDR_INCLUDE_DIR}
)

target_link_libraries(voxblox
    Eigen3::Eigen
    Threads::Threads
    ${PROTOBUF_LIBRARIES}
)

# Add glog/gflags if found
if(GLOG_FOUND)
    target_include_directories(voxblox PRIVATE ${GLOG_INCLUDE_DIRS})
    target_link_libraries(voxblox ${GLOG_LIBRARIES})
    target_compile_definitions(voxblox PRIVATE -DVOXBLOX_USE_GLOG)
endif()

if(GFLAGS_FOUND)
    target_include_directories(voxblox PRIVATE ${GFLAGS_INCLUDE_DIRS})
    target_link_libraries(voxblox ${GFLAGS_LIBRARIES})
endif()

# Compile definitions
target_compile_definitions(voxblox PRIVATE 
    -DVOXBLOX_STANDALONE
)
