/**
 * @file clustering.h
 * @brief Voxel-based clustering for dynamic detection (based on original)
 */

#ifndef DYNABLOX_CLUSTERING_H_
#define DYNABLOX_CLUSTERING_H_

#include <memory>
#include <vector>

#include "dynablox/types.h"
#include "dynablox/config.h"
#include "dynablox/neighborhood_search.h"

namespace dynablox {

/**
 * @brief Voxel-based clustering using Voxblox TsdfLayer
 */
class Clustering {
public:
    using Ptr = std::shared_ptr<Clustering>;
    using ClusterIndices = std::vector<voxblox::VoxelKey>;
    
    Clustering(const ClusteringConfig& config, TsdfLayer* tsdf_layer);
    
    /**
     * @brief Cluster occupied ever-free voxels
     * @param cloud Point cloud (global frame)
     * @param point_map Map of blocks to voxels to points
     * @param occupied_ever_free_voxels Seed voxels for clustering
     * @param cloud_info Output: point info with dynamic flags
     * @return Found clusters
     */
    Clusters performClustering(
        const Pointcloud& cloud,
        const BlockToPointMap& point_map,
        const ClusterIndices& occupied_ever_free_voxels,
        CloudInfo& cloud_info) const;
    
    /**
     * @brief Voxel-level clustering
     */
    std::vector<ClusterIndices> voxelClustering(
        const ClusterIndices& occupied_ever_free_voxels,
        int frame_counter) const;
    
    /**
     * @brief Grow a single cluster from seed
     */
    bool growCluster(const voxblox::VoxelKey& seed,
                     int frame_counter,
                     ClusterIndices& result) const;
    
    /**
     * @brief Convert voxel clusters to point clusters
     */
    Clusters inducePointClusters(
        const BlockToPointMap& point_map,
        const std::vector<ClusterIndices>& voxel_clusters) const;
    
    /**
     * @brief Apply cluster-level filters
     */
    void applyClusterLevelFilters(Clusters& clusters) const;
    
    /**
     * @brief Check if cluster should be filtered out
     */
    bool filterCluster(const Cluster& cluster) const;
    
    /**
     * @brief Set dynamic flags on points
     */
    void setClusterLevelDynamicFlag(const Clusters& clusters,
                                    CloudInfo& cloud_info) const;

private:
    ClusteringConfig config_;
    TsdfLayer* tsdf_layer_;
    NeighborhoodSearch neighborhood_search_;
    
    float voxel_size_;
    size_t voxels_per_side_;
};

}  // namespace dynablox

#endif  // DYNABLOX_CLUSTERING_H_
