/**
 * @file clustering.cpp
 * @brief Voxel-based clustering implementation (based on original Dynablox)
 */

#include "dynablox/clustering.h"

#include <queue>
#include <limits>

#include <voxblox/integrator/integrator_utils.h>

namespace dynablox {

Clustering::Clustering(const ClusteringConfig& config, TsdfLayer* tsdf_layer)
    : config_(config),
      tsdf_layer_(tsdf_layer),
      neighborhood_search_(config.neighbor_connectivity),
      voxel_size_(tsdf_layer->voxel_size()),
      voxels_per_side_(tsdf_layer->voxels_per_side()) {
}

Clusters Clustering::performClustering(
    const Pointcloud& cloud,
    const BlockToPointMap& point_map,
    const ClusterIndices& occupied_ever_free_voxels,
    CloudInfo& cloud_info) const {
    
    // Reset clustering_processed flags
    voxblox::BlockIndexList blocks;
    tsdf_layer_->getAllAllocatedBlocks(&blocks);
    for (const auto& block_idx : blocks) {
        auto block = tsdf_layer_->getBlockPtrByIndex(block_idx);
        if (!block) continue;
        
        size_t num_voxels = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
        for (size_t i = 0; i < num_voxels; ++i) {
            block->getVoxelByLinearIndex(i).clustering_processed = false;
        }
    }
    
    // Voxel-level clustering (use current frame as implicit frame counter)
    auto voxel_clusters = voxelClustering(occupied_ever_free_voxels, 0);
    
    // Convert to point clusters
    Clusters clusters = inducePointClusters(point_map, voxel_clusters);
    
    // Apply filters
    applyClusterLevelFilters(clusters);
    
    // Set point-level dynamic flags
    setClusterLevelDynamicFlag(clusters, cloud_info);
    
    return clusters;
}

std::vector<Clustering::ClusterIndices> Clustering::voxelClustering(
    const ClusterIndices& occupied_ever_free_voxels,
    int frame_counter) const {
    
    std::vector<ClusterIndices> voxel_clusters;
    
    for (const auto& voxel_key : occupied_ever_free_voxels) {
        ClusterIndices cluster;
        if (growCluster(voxel_key, frame_counter, cluster)) {
            voxel_clusters.push_back(cluster);
        }
    }
    
    return voxel_clusters;
}

bool Clustering::growCluster(const voxblox::VoxelKey& seed,
                             int frame_counter,
                             ClusterIndices& result) const {
    
    std::vector<voxblox::VoxelKey> stack = {seed};
    
    while (!stack.empty()) {
        voxblox::VoxelKey voxel_key = stack.back();
        stack.pop_back();
        
        auto tsdf_block = tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
        if (!tsdf_block) continue;
        
        TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_key.second);
        
        // Process each voxel only once
        if (tsdf_voxel.clustering_processed) continue;
        
        // Add to cluster
        tsdf_voxel.dynamic = true;
        tsdf_voxel.clustering_processed = true;
        result.push_back(voxel_key);
        
        // Extend to neighbors
        auto neighbors = neighborhood_search_.search(
            voxel_key.first, voxel_key.second, voxels_per_side_);
        
        for (const auto& neighbor_key : neighbors) {
            auto neighbor_block = tsdf_layer_->getBlockPtrByIndex(neighbor_key.first);
            if (!neighbor_block) continue;
            
            TsdfVoxel& neighbor_voxel = 
                neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);
            
            // Valid neighbor: not processed, occupied this frame
            if (!neighbor_voxel.clustering_processed &&
                neighbor_voxel.last_lidar_occupied == frame_counter) {
                
                // Grow from ever-free voxels, or one step from current if configured
                if (neighbor_voxel.ever_free ||
                    (tsdf_voxel.ever_free && config_.grow_clusters_twice)) {
                    stack.push_back(neighbor_key);
                } else {
                    // Add to cluster but don't grow further
                    neighbor_voxel.dynamic = true;
                    neighbor_voxel.clustering_processed = true;
                    result.push_back(neighbor_key);
                }
            }
        }
    }
    
    return !result.empty();
}

Clusters Clustering::inducePointClusters(
    const BlockToPointMap& point_map,
    const std::vector<ClusterIndices>& voxel_clusters) const {
    
    Clusters clusters;
    int cluster_id = 0;
    
    for (const auto& voxel_cluster : voxel_clusters) {
        Cluster cluster;
        cluster.id = cluster_id++;
        cluster.voxel_keys = voxel_cluster;
        
        // Initialize AABB
        cluster.min_bound = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max());
        cluster.max_bound = Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                                            std::numeric_limits<float>::lowest(),
                                            std::numeric_limits<float>::lowest());
        
        for (const auto& voxel_key : voxel_cluster) {
            // Find block in point map
            auto block_it = point_map.find(voxel_key.first);
            if (block_it == point_map.end()) continue;
            
            // Find voxel in block
            auto voxel_it = block_it->second.find(voxel_key.second);
            if (voxel_it == block_it->second.end()) continue;
            
            // Get voxel center
            voxblox::GlobalIndex global_idx = 
                voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
                    voxel_key.first, voxel_key.second, voxels_per_side_);
            Point center = voxblox::getCenterPointFromGridIndex(global_idx, voxel_size_);
            
            // Add all point indices
            for (int point_idx : voxel_it->second) {
                cluster.point_indices.push_back(static_cast<size_t>(point_idx));
            }
            
            // Update AABB from voxel center
            float half_voxel = voxel_size_ * 0.5f;
            cluster.min_bound = cluster.min_bound.cwiseMin(
                center - Eigen::Vector3f(half_voxel, half_voxel, half_voxel));
            cluster.max_bound = cluster.max_bound.cwiseMax(
                center + Eigen::Vector3f(half_voxel, half_voxel, half_voxel));
        }
        
        // Compute center
        cluster.center = (cluster.min_bound + cluster.max_bound) * 0.5f;
        
        if (!cluster.point_indices.empty()) {
            clusters.push_back(cluster);
        }
    }
    
    return clusters;
}

void Clustering::applyClusterLevelFilters(Clusters& clusters) const {
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                      [this](const Cluster& c) { return filterCluster(c); }),
        clusters.end());
    
    // Mark valid clusters
    for (auto& cluster : clusters) {
        cluster.valid = true;
    }
}

bool Clustering::filterCluster(const Cluster& cluster) const {
    // Size filter
    size_t num_points = cluster.point_indices.size();
    if (num_points < static_cast<size_t>(config_.min_cluster_size) ||
        num_points > static_cast<size_t>(config_.max_cluster_size)) {
        return true;
    }
    
    // Extent filter
    float extent = cluster.extent();
    if (extent < config_.min_extent || extent > config_.max_extent) {
        return true;
    }
    
    return false;
}

void Clustering::setClusterLevelDynamicFlag(const Clusters& clusters,
                                            CloudInfo& cloud_info) const {
    for (const auto& cluster : clusters) {
        if (!cluster.valid) continue;
        
        for (size_t point_idx : cluster.point_indices) {
            if (point_idx < cloud_info.points.size()) {
                cloud_info.points[point_idx].is_dynamic = true;
                cloud_info.points[point_idx].cluster_id = cluster.id;
            }
        }
    }
}

}  // namespace dynablox
