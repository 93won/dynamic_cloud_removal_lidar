/**
 * @file ever_free_integrator.h
 * @brief Ever-Free Integrator using Voxblox TsdfLayer (based on original)
 */

#ifndef DYNABLOX_EVER_FREE_INTEGRATOR_H_
#define DYNABLOX_EVER_FREE_INTEGRATOR_H_

#include <memory>
#include <vector>
#include <future>
#include <mutex>

#include <voxblox/core/layer.h>
#include <voxblox/core/block.h>

#include "dynablox/types.h"
#include "dynablox/config.h"
#include "dynablox/neighborhood_search.h"

namespace dynablox {

/**
 * @brief Ever-Free Integrator (original Dynablox logic)
 * 
 * Uses Voxblox TsdfLayer with extended TsdfVoxel fields:
 * - ever_free: Whether voxel is considered free space
 * - last_lidar_occupied: Last frame a LiDAR point fell in this voxel
 * - last_occupied: Last frame voxel was occupied (by TSDF or LiDAR)
 * - occ_counter: Number of consecutive occupied frames
 * - dynamic: Whether voxel contains dynamic object
 */
class EverFreeIntegrator {
public:
    using Ptr = std::shared_ptr<EverFreeIntegrator>;
    
    EverFreeIntegrator(const EverFreeConfig& config, TsdfLayer* tsdf_layer);
    
    /**
     * @brief Update ever-free state of all changed TSDF voxels
     * @param frame_counter Current frame index
     */
    void updateEverFreeVoxels(int frame_counter) const;
    
    /**
     * @brief Process a block in parallel
     * @param block_index Block to process
     * @param frame_counter Current frame
     * @param voxels_to_remove Output: voxels outside block needing cleanup
     * @return True if any voxels need removal
     */
    bool blockWiseUpdateEverFree(
        const BlockIndex& block_index,
        int frame_counter,
        voxblox::AlignedVector<voxblox::VoxelKey>& voxels_to_remove) const;
    
    /**
     * @brief Update occupancy counter for a voxel
     */
    void updateOccupancyCounter(TsdfVoxel& voxel, int frame_counter) const;
    
    /**
     * @brief Remove ever-free from voxel and neighbors
     */
    voxblox::AlignedVector<voxblox::VoxelKey> removeEverFree(
        TsdfBlock& block,
        TsdfVoxel& voxel,
        const BlockIndex& block_index,
        const VoxelIndex& voxel_index) const;
    
    /**
     * @brief Check and label voxels as ever-free
     */
    void blockWiseMakeEverFree(const BlockIndex& block_index,
                               int frame_counter) const;

private:
    EverFreeConfig config_;
    TsdfLayer* tsdf_layer_;
    NeighborhoodSearch neighborhood_search_;
    
    // Cached values
    float voxel_size_;
    size_t voxels_per_side_;
    size_t voxels_per_block_;
};

}  // namespace dynablox

#endif  // DYNABLOX_EVER_FREE_INTEGRATOR_H_
