/**
 * @file types.h
 * @brief Core types for Dynablox (ROS-free, uses Voxblox directly)
 */

#ifndef DYNABLOX_TYPES_H_
#define DYNABLOX_TYPES_H_

#include <vector>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>
#include <voxblox/core/common.h>
#include <voxblox/core/block.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

namespace dynablox {

// Voxblox types
using VoxelIndex = voxblox::VoxelIndex;
using BlockIndex = voxblox::BlockIndex;
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using Pointcloud = voxblox::Pointcloud;
using Point = voxblox::Point;
using Transformation = voxblox::Transformation;

// Maps each voxel in a block to point cloud indices that fall into it
using VoxelToPointMap = voxblox::HierarchicalIndexIntMap;

// Map of block indices to voxel-to-point maps  
using BlockToPointMap = voxblox::AnyIndexHashMapType<VoxelToPointMap>::type;

// Cloud info point for clustering
struct CloudInfoPoint {
    size_t point_index = 0;
    size_t original_index = 0;  // Index in original cloud before filtering
    float distance = 0.0f;
    bool is_dynamic = false;
    int cluster_id = -1;
};

// Cloud info for processing
struct CloudInfo {
    std::vector<CloudInfoPoint> points;
};

// Cluster of dynamic points
struct Cluster {
    int id = -1;
    int track_length = 0;
    bool valid = true;
    
    // Bounding box (global frame)
    Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    
    // Voxels in this cluster
    std::vector<voxblox::VoxelKey> voxel_keys;
    
    // Point indices (into filtered cloud)
    std::vector<size_t> point_indices;
    
    float extent() const {
        return (max_bound - min_bound).norm();
    }
};

using Clusters = std::vector<Cluster>;

// Detection result for a single frame
struct DetectionResult {
    uint32_t frame_id = 0;
    double timestamp = 0.0;
    
    // Point-level results (indices into original cloud)
    std::vector<size_t> dynamic_indices;
    std::vector<size_t> static_indices;
    
    // Cluster results
    Clusters clusters;
    
    // Statistics
    size_t num_total = 0;
    size_t num_filtered = 0;
    size_t num_dynamic = 0;
    size_t num_clusters = 0;
    size_t num_ever_free_seeds = 0;
    
    float dynamic_ratio() const {
        return num_filtered > 0 ? static_cast<float>(num_dynamic) / num_filtered : 0.0f;
    }
};

}  // namespace dynablox

#endif  // DYNABLOX_TYPES_H_
